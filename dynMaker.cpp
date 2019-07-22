
#include <fstream>
#include <iostream>
#include <string>
#include <map>
#include <sstream>
#include <vector>
#include "Quaternion.h"
#include "Animate.h"

std::string filename("mushroom");
std::ifstream infile;
std::map<int, std::string> animationFiles;
int hashCount;
std::map<int, int> hashmap;
std::vector<float> allData;
Matrix4f bindShapeMatrix;
std::map<std::string, int> jointIDMap;
std::vector<Matrix4f> iInverseBindPose;
int noOfJoints;
int rootBoneID;
std::vector<int>* heirarchyOfBone;
Animate animate;
std::vector<Vector3f> jID, wID;
std::vector<int> indicies;
std::map<int, Matrix4f> localToWorldMatrix;

void convertMatrix(Matrix4f & matrix, Quaternion & quaternion, Vector3f & vector)
{

	float sq[4];
	// 11 =0 22=5 33=10
	sq[0] = matrix[0][0] + matrix[1][1] + matrix[2][2];
	sq[1] = matrix[0][0] - matrix[1][1] - matrix[2][2];
	sq[2] = -matrix[0][0] + matrix[1][1] - matrix[2][2];
	sq[3] = -matrix[0][0] - matrix[1][1] + matrix[2][2];
	int gIndex = -1;	float g = -99999;
	for (int i = 0; i < 4; i++)
	{
		if (sq[i] > g)
		{
			g = sq[i];
			gIndex = i;
		}
	}
	float w, x, y, z;
	switch (gIndex)
	{
	case 0:	// w greatest
		w = std::sqrt(sq[0] + 1) / 2;
		x = (matrix[1][2] - matrix[2][1]) / (4 * w);
		y = (matrix[2][0] - matrix[0][2]) / (4 * w);
		z = (matrix[0][1] - matrix[1][0]) / (4 * w);
		break;
	case 1: // x greatest
		x = std::sqrt(sq[1] + 1) / 2;
		w = (matrix[1][2] - matrix[2][1]) / (4 * x);
		y = (matrix[0][1] + matrix[1][0]) / (4 * x);
		z = (matrix[0][2] + matrix[2][0]) / (4 * x);
		break;
	case 2:	// y greatest
		y = std::sqrt(sq[2] + 1) / 2;
		w = (matrix[2][0] - matrix[0][2]) / (4 * y);
		x = (matrix[1][0] + matrix[0][1]) / (4 * y);
		z = (matrix[2][1] + matrix[1][2]) / (4 * y);
		break;
	case 3:
		z = std::sqrt(sq[3] + 1) / 2;
		w = (matrix[0][1] - matrix[1][0]) / (4 * z);
		x = (matrix[2][0] + matrix[0][2]) / (4 * z);
		y = (matrix[2][1] + matrix[1][2]) / (4 * z);
		break;
	}


	quaternion = Quaternion(w, x, y, z);
	vector.x(matrix[3][0]);	vector.y(matrix[3][1]);	vector.z(matrix[3][2]);
}

bool allow(int x, int y, int z, int* indi)
{
	bool go = true;
	int hash = x * 10000 + y * y*10 + z;
	if (hashmap.count(hash) <= 0 || hashCount == 0)
		hashmap.insert(std::pair<int, int>(hash, hashCount++));
	else
		go = false;

	if (hashmap[hash] < hashCount)
		*indi = hashmap[hash];
	else
		*indi = hashCount;

	return go;
}

std::streampos geometriesLoc()
{
	std::string line;
	std::streampos pos;
	while (std::getline(infile, line))
	{
		if (line.find("<library_geometries>") != std::string::npos)
		{
			pos = infile.tellg();
			break;
		}
	}
	return pos;
}

void geometries(std::streampos pos)
{
	std::string line;
	std::vector<float> vertex, normal, texCoord;

	infile.seekg(pos);

	int vnt = 0;
	while (std::getline(infile, line))
	{
		// vertices
		if (vnt == 0 && line.find("<float_array") != std::string::npos)
		{
			size_t start, end;
			start = line.find(">"); end = line.find("<", start + 1);
			line = line.substr(start + 1, end - start - 1);
			std::istringstream iss(line);
			float x, y, z;
			while (iss >> x >> y >> z)
			{
				vertex.push_back(x);
				vertex.push_back(y);
				vertex.push_back(z);
			}
			vnt++;
		}

		//normals
		if (vnt == 1 && line.find("<float_array") != std::string::npos)
		{
			size_t start, end;
			start = line.find(">"); end = line.find("<", start + 1);
			line = line.substr(start + 1, end - start - 1);
			std::istringstream iss(line);
			float x, y, z;
			while (iss >> x >> y >> z)
			{
				normal.push_back(x);
				normal.push_back(y);
				normal.push_back(z);
			}
			vnt++;
		}

		//texcoords
		if (vnt == 2 && line.find("<float_array") != std::string::npos)
		{
			size_t start, end;
			start = line.find(">"); end = line.find("<", start + 1);
			line = line.substr(start + 1, end - start - 1);
			std::istringstream iss(line);
			float x, y;
			while (iss >> x >> y)
			{
				texCoord.push_back(x);
				texCoord.push_back(y);
			}
			vnt++;
		}


		//finally fill data according to indices
		if (line.find("<p>") != std::string::npos)
		{
			size_t start, end;
			start = line.find(">");
			end = line.find("<", start + 1);
			line = line.substr(start + 1, end - (start + 1));
			std::istringstream iss(line);

			int v, n, t;
			while (iss >> v >> n >> t)
			{
				int index = -1;
				if (allow(v, n, t, &index))
				{
					allData.push_back(vertex[3 * v]);
					allData.push_back(vertex[3 * v + 1]);
					allData.push_back(vertex[3 * v + 2]);

					allData.push_back(texCoord[2 * t]);
					allData.push_back(texCoord[2 * t + 1]);

					allData.push_back(normal[3 * n]);
					allData.push_back(normal[3 * n + 1]);
					allData.push_back(normal[3 * n + 2]);

					allData.push_back(jID[v].x());
					allData.push_back(jID[v].y());
					allData.push_back(jID[v].z());

					allData.push_back(wID[v].x());
					allData.push_back(wID[v].y());
					allData.push_back(wID[v].z());
				}
				indicies.push_back(index);
			}
		}

		if (line.find("</library_geometries>") != std::string::npos)
			break;
	}

}

void controllers()
{
	std::string line;
	std::vector<float> weights;
	std::vector<int> vcount;

	while (std::getline(infile, line))
	{
		if (line.find("<library_controllers>") != std::string::npos)
			break;
	}
	while (std::getline(infile, line))
	{
		if (line.find("</library_controllers>") != std::string::npos)
			break;
		if (line.find("<bind_shape_matrix>") != std::string::npos)
		{
			size_t start, end;
			start = line.find(">");		end = line.find("<", start + 1);
			line = line.substr(start + 1, end - start - 1);
			std::istringstream iss(line);
			float f;
			int col = 0, row = 0;
			while (iss >> f)
			{
				bindShapeMatrix[col++][row] = f;
				if (col % 4 == 0)
				{
					col = 0;
					row++;
				}
			}
		}
		
		if (line.find("<Name_array") != std::string::npos)
		{
			size_t s = line.find(">");
			size_t f = line.find("<", s + 1);
			line = line.substr(s+1, f - s - 1);
			std::istringstream iss(line);
			int id = 0;
			std::string somedata;
			while (iss >> somedata)
			{
				std::pair<std::string, int> jidpair(somedata, id++);
				jointIDMap.insert(jidpair);
			}
			noOfJoints = id;

		}
		
		if (line.find("<float_array") != std::string::npos)
		{
			size_t s = line.find(">");
			size_t e = line.find("<", s + 1);
			line = line.substr(s+1, e - s - 1);
			std::istringstream iss(line);
			float f; 
			int col=0, row=0;
			Matrix4f IBPi;
			while (iss >> f)
			{
				IBPi[col++][row] = f;
				if (col == 4)
				{
					col = 0;
					row++;
				}
				if (row == 4)
				{
					col = 0;
					row = 0;
					iInverseBindPose.push_back(IBPi);
				}
			}
			break;
		}
	}

	while (std::getline(infile, line))
	{
		if (line.find("</library_controllers>") != std::string::npos)
			break;
		if (line.find("<float_array") != std::string::npos)
		{
			size_t s = line.find(">");
			size_t e = line.find("<", s + 1);
			line = line.substr(s+1, e - s - 1);
			std::istringstream iss(line);
			float f;
			while (iss >> f)
			{
				weights.push_back(f);
			}
		}
		
		if (line.find("<vcount>") != std::string::npos)
		{
			size_t s = line.find(">");
			size_t e = line.find("<", s + 1);
			line = line.substr(s+1, e - s - 1);
			std::istringstream iss(line);
			int i;
			while (iss >> i)
			{
				vcount.push_back(i);
			}

		}
		
		if (line.find("<v>") != std::string::npos)
		{
			size_t s = line.find(">");
			size_t e = line.find("<", s + 1);
			line = line.substr(s+1, e - s - 1);
			std::istringstream iss(line);
			for (int i = 0; i < vcount.size(); i++)
			{
				if (vcount[i] == 1)
				{
					int j, w;
					iss >> j >> w;
					jID.push_back(Vector3f(j, -1, -1));
					wID.push_back(Vector3f(weights[w], 0, 0));
				}
				else if (vcount[i] == 2)
				{
					int j0, j1, w0, w1;
					iss >> j0 >> w0 >> j1 >> w1;
					jID.push_back(Vector3f(j0, j1, -1));
					wID.push_back(Vector3f(weights[w0], weights[w1], 0));
				}
				else if (vcount[i] == 3)
				{
					int j0, j1, j2, w0, w1, w2;
					iss >> j0 >> w0 >> j1 >> w1 >> j2 >> w2;
					jID.push_back(Vector3f(j0, j1, j2));
					wID.push_back(Vector3f(weights[w0], weights[w1], weights[w2]));
				}
				else
				{
					int j0 = -1, j1 = -1, j2 = -1;
					float w0 = -1, w1 = -1, w2 = -1;	//in this case w0,w1,w2 will be direct weight values and are not indices of weights
					for (int x = 0; x < vcount[i]; x++)
					{
						int j, w;
						iss >> j >> w;
						if (weights[w] > w0)
						{
							w2 = w1;
							j2 = j1;
							w1 = w0;
							j1 = j0;
							w0 = weights[w];
							j0 = j;
						}
						else if (weights[w] > w1)
						{
							w2 = w1;
							j2 = j1;
							w1 = weights[w];
							j1 = j;
						}
						else if (weights[w] > w2)
						{
							w2 = weights[w];
							j2 = j;
						}
					}

					jID.push_back(Vector3f(j0, j1, j2));
					wID.push_back(Vector3f(w0, w1, w2));
				}
			}
		}

		
	}

}

std::streampos animationsLoc()
{
	std::string line;
	std::streampos pos;
	while (std::getline(infile, line))
	{
		if (line.find("<library_animations>") != std::string::npos)
		{
			pos = infile.tellg();
			break;
		}
	}
	return pos;
}

void animations(std::streampos pos)
{
	infile.seekg(pos);

	Animations singleAnimation;

	std::string line;
	
	int state =0;
	bool  initialize = true;
	int frame = 0;
	while (std::getline(infile, line))
	{
		if (line.find("</library_animations>") != std::string::npos)
			break;
		if (state == 0 && line.find("<float_array id=\"Armature_") != std::string::npos)	//state 0 finding no of keyframes
		{
			if (initialize)
			{
				size_t start = line.find(">");
				size_t fin = line.find("<", start + 1);
				line = line.substr(start + 1, fin - start - 1);
				float f;	int t=0;
				std::istringstream iss(line);
				while (iss >> f)
				{
					t++;
				}
				singleAnimation.noOfKeyFrames = t;
				singleAnimation.frames = new KeyFrames[singleAnimation.noOfKeyFrames];
				for (int x = 0; x < singleAnimation.noOfKeyFrames; x++)
				{
					singleAnimation.frames[x].jData = new Matrix4f[noOfJoints];
					singleAnimation.frames[x].rotation = new Quaternion[noOfJoints];
					singleAnimation.frames[x].translate = new Vector3f[noOfJoints];
				}
				initialize = false;
			}
			state = 1;
		}
		else if (state == 1 && line.find("<float_array id=\"Armature_") != std::string::npos)
		{
			int index = 0; 
			std::string nameOfBone="";
			size_t start = line.find("<float_array id=\"Armature_");
			std::string len = "<float_array id=\"Armature_";
			int size = len.length();
			size_t finish = line.find("_pose_matrix-output-array");
			nameOfBone = line.substr(start + size , finish - start - size );
			index = jointIDMap[nameOfBone];
			start = line.find(">");
			finish = line.find("<", start + 1);
			line = line.substr(start + 1, finish - start - 1);
			std::istringstream iss(line);
			float f; int col=0,row=0;
			Matrix4f temp;
			int counter=0;
			
				while (iss >> f)
				{
					temp[col++][row] = f;
					if (col == 4)
					{
						col = 0;
						row++;
					}
					if (row == 4)
					{
						col = 0;
						row = 0;
						singleAnimation.frames[frame].jData[index] = temp;
						frame++;
						if (frame >= singleAnimation.noOfKeyFrames)
						{
							frame = 0;
						}
					}
				}
			
			state = 0;

		}
	}

	animate.animations.push_back(singleAnimation);
}

void multiplyTransformsHeirarchicaly(int id, int i, int j)
{
	for (int k = 0; k < heirarchyOfBone[id].size(); k++)
	{
		animate.animations[i].frames[j].jData[heirarchyOfBone[id][k]] = animate.animations[i].frames[j].jData[id] * animate.animations[i].frames[j].jData[heirarchyOfBone[id][k]];
		multiplyTransformsHeirarchicaly(heirarchyOfBone[id][k], i, j);
	}
}

void multiply()
{
	Matrix4f to90;
	to90.identity();
	to90[1][1] = 0;		to90[1][2] = -1;
	to90[2][1] = 1;		to90[2][2] = 0;

	for (int i = 0; i < animate.noOfAnimations; i++)
	{
		for (int j = 0; j < animate.animations[i].noOfKeyFrames; j++)
		{
			animate.animations[i].frames[j].jData[rootBoneID] = to90 * animate.animations[i].frames[j].jData[rootBoneID];
		}
	}

	for (int i = 0; i < animate.noOfAnimations; i++)
	{
		for (int j = 0; j < animate.animations[i].noOfKeyFrames; j++)
		{

			multiplyTransformsHeirarchicaly(rootBoneID, i, j);
			for (int k = 0; k < noOfJoints; k++)
			{
				Matrix4f temp = animate.animations[i].frames[j].jData[k] * iInverseBindPose[k] * bindShapeMatrix;
				convertMatrix(temp, animate.animations[i].frames[j].rotation[k], animate.animations[i].frames[j].translate[k]);
			}
		}
	}
}

void createHeirarchy(int parentID, Matrix4f& mat)
{
	std::string line;
	while (std::getline(infile, line))
	{
		if (line.find("<node id") != std::string::npos)
		{
			size_t start = line.find("name=\"");
			size_t end = line.find("\"", start + 6);
			line = line.substr(start + 6, end - (start + 6));
			if (line.compare("Armature") == 0)
				continue;
			int id = jointIDMap[line];
			if (parentID < 0)
				rootBoneID = id;
			
			Matrix4f trans;
			std::getline(infile, line);
			start = line.find(">");
			end = line.find("<", start + 1);
			line = line.substr(start + 1, end - start - 1);
			std::istringstream iss(line);
			float f;
			int col = 0, row = 0;
			while (iss >> f)
			{
				trans[row][col++] = f;
				if (col % 4 == 0)
				{
					col = 0;
					row++;
				}
			}
			trans = mat * trans;	
			localToWorldMatrix.insert(std::pair<int, Matrix4f>(id, trans));
			if(parentID >= 0)
				heirarchyOfBone[parentID].push_back(id);
			createHeirarchy(id, trans);
		}
		if (line.find("</node>") != std::string::npos)
			return;
			
	}
}

void visualScenes()
{
	std::string line;

	while (std::getline(infile, line))
	{
		if (line.find("<library_visual_scenes>") != std::string::npos)
		{
			break;
		}
	}
	while (std::getline(infile, line))
	{
		if (line.find("name=\"Armature\" type=\"NODE\"") != std::string::npos)
		{
			break;
		}	
	}
	while (std::getline(infile, line))
	{
		heirarchyOfBone = new std::vector<int>[noOfJoints];
		Matrix4f trans;
		trans.identity();
		createHeirarchy(-1,trans);
		break;
	}
	
}

void writeToFile()
{
	std::ofstream ofile(filename + ".dyn");
	for (int i = 0; i < allData.size(); i++)
	{
		ofile << allData[i] << " ";
	}
	ofile << std::endl;
	for (int i = 0; i < indicies.size(); i++)
	{
		ofile << indicies[i] << " ";
	}
	ofile << std::endl;
	ofile << noOfJoints << std::endl;
	for (std::map<std::string, int>::iterator i = jointIDMap.begin(); i != jointIDMap.end(); i++)
	{
		ofile << i->first << " " << i->second << std::endl;
	}
	ofile << animate.noOfAnimations << std::endl;
	for (int i = 0; i < animate.noOfAnimations; i++)
	{
		ofile << animationFiles[i] << " ";
		ofile << animate.animations[i].noOfKeyFrames << std::endl;
		for (int j = 0; j < animate.animations[i].noOfKeyFrames; j++)
		{
			for (int k = 0; k < noOfJoints; k++)
			{
				ofile << animate.animations[i].frames[j].rotation[k] << " " << animate.animations[i].frames[j].translate[k] << " ";
			}
			ofile << std::endl;
		}
	}

}

int main()
{
	int no = 0;
	bool init = true;

	animationFiles.insert(std::pair<int, std::string>(no++, "melee"));
	animationFiles.insert(std::pair<int, std::string>(no++, "ranged"));

	animate.noOfAnimations = no;


	std::streampos geoPos;
	for (int a = 0; a < no; a++)
	{
		infile = std::ifstream(filename +"_"+ animationFiles[a] + ".dae");
		if(init)
			geoPos = geometriesLoc();
		std::streampos animationsPos = animationsLoc();
		if(init)
			controllers();
		animations(animationsPos);
		if(init)
			visualScenes();
		if(init)
			geometries(geoPos);
		if (init)
			init = false;
	}
	multiply();
	writeToFile();
	delete[] heirarchyOfBone;		//its in main thread, so this statement is not required
	return 0;
}
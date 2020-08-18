#include "skinning.h"
#include "vec3d.h"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <fstream>
using namespace std;

// CSCI 520 Computer Animation and Simulation
// Jernej Barbic and Yijing Li

Skinning::Skinning(int numMeshVertices, const double * restMeshVertexPositions,
    const std::string & meshSkinningWeightsFilename)
{
    this->numMeshVertices = numMeshVertices;
    this->restMeshVertexPositions = restMeshVertexPositions;

    cout << "Loading skinning weights..." << endl;
    ifstream fin(meshSkinningWeightsFilename.c_str());
    assert(fin);
    int numWeightMatrixRows = 0, numWeightMatrixCols = 0;
    fin >> numWeightMatrixRows >> numWeightMatrixCols;
    assert(fin.fail() == false);
    assert(numWeightMatrixRows == numMeshVertices);
    int numJoints = numWeightMatrixCols;

    vector<vector<int>> weightMatrixColumnIndices(numWeightMatrixRows);
    vector<vector<double>> weightMatrixEntries(numWeightMatrixRows);
    fin >> ws;
    while(fin.eof() == false)
    {
        int rowID = 0, colID = 0;
        double w = 0.0;
        fin >> rowID >> colID >> w;
        weightMatrixColumnIndices[rowID].push_back(colID);
        weightMatrixEntries[rowID].push_back(w);
        assert(fin.fail() == false);
        fin >> ws;
    }
    fin.close();

    // Build skinning joints and weights.
    numJointsInfluencingEachVertex = 0;
    for (int i = 0; i < numMeshVertices; i++)
        numJointsInfluencingEachVertex = std::max(numJointsInfluencingEachVertex, (int)weightMatrixEntries[i].size());
    assert(numJointsInfluencingEachVertex >= 2);

    // Copy skinning weights from SparseMatrix into meshSkinningJoints and meshSkinningWeights.
    meshSkinningJoints.assign(numJointsInfluencingEachVertex * numMeshVertices, 0);
    meshSkinningWeights.assign(numJointsInfluencingEachVertex * numMeshVertices, 0.0);
    for (int vtxID = 0; vtxID < numMeshVertices; vtxID++)
    {
    vector<pair<double, int>> sortBuffer(numJointsInfluencingEachVertex);
    for (size_t j = 0; j < weightMatrixEntries[vtxID].size(); j++)
    {
        int frameID = weightMatrixColumnIndices[vtxID][j];
        double weight = weightMatrixEntries[vtxID][j];
        sortBuffer[j] = make_pair(weight, frameID);
    }
    sortBuffer.resize(weightMatrixEntries[vtxID].size());
    assert(sortBuffer.size() > 0);
    sort(sortBuffer.rbegin(), sortBuffer.rend()); // sort in descending order using reverse_iterators
    for(size_t i = 0; i < sortBuffer.size(); i++)
    {
      meshSkinningJoints[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].second;
      meshSkinningWeights[vtxID * numJointsInfluencingEachVertex + i] = sortBuffer[i].first;
    }

    // Note: When the number of joints used on this vertex is smaller than numJointsInfluencingEachVertex,
    // the remaining empty entries are initialized to zero due to vector::assign(XX, 0.0) .
    }
}

void Skinning::applySkinning(const RigidTransform4d * jointSkinTransforms, double * newMeshVertexPositions) const
{
    // Students should implement this
    //newMeshVertexPositions in Vec3d formay
    Vec3d vecNewResult = Vec3d(0.0, 0.0, 0.0);
      
    for(int i = 0; i < numMeshVertices; i++)
    {
        vecNewResult.set(0.0);
        
        //restMeshVertexPosition in Vec3d format
        Vec3d vecRestPosition = Vec3d(restMeshVertexPositions[3 * i], restMeshVertexPositions[3 * i + 1], restMeshVertexPositions[3 * i + 2]);
        //wj * Mj * ba(pi)
        for(int j = 0; j < numJointsInfluencingEachVertex; j++)
        {
            
            //AffineTransform4d::transformPoint(const Vec3d & p) const
            vecNewResult += meshSkinningWeights[numJointsInfluencingEachVertex * i + j] * jointSkinTransforms[meshSkinningJoints[numJointsInfluencingEachVertex * i + j]].transformPoint(vecRestPosition);
        }
        newMeshVertexPositions[3 * i] = vecNewResult[0];
        newMeshVertexPositions[3 * i + 1] = vecNewResult[1];
        newMeshVertexPositions[3 * i + 2] = vecNewResult[2];
    }
    
}


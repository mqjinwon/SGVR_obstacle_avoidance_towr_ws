#include <iostream>
#include <towr/terrain/height_map.h>
#include <towr/terrain/grid_map.h>

#include <fstream>
#include <string.h>

using namespace std;

int main(void){

    towr::HeightMap::TerrainID terrain_id = towr::HeightMap::TerrainID::RandomStairsID;
    std::string gridLayerName_ = "grid_map"; 
    SGVR::TowrMap::Ptr terrain_ = std::make_shared<SGVR::TowrMap>();

    terrain_->setHeightMap(towr::HeightMap::MakeTerrain(terrain_id));

    terrain_->addGridMapLayer(gridLayerName_);
    double dxy   =  0.1; // 1cm resolution
    double x_min =  0.9;
    double x_max =  1.1;
    double y_min =  -0.3;
    double y_max =  0.3;

    terrain_->gridMapInit("world", 
                       grid_map::Length(x_max - x_min, y_max - y_min), dxy, 
                       grid_map::Position((x_min+x_max)/2.0, (y_min+y_max)/2.0));
    terrain_->updateGridMap(gridLayerName_);   //update grid map from height map information
    terrain_->updateSDF(gridLayerName_, 0.1);  //update sdf from grid map information, margin z=0.1

    // for(double x=0; x<2; x += 0.01){
    //     for(double y=0.01; y<0.02; y += 0.01){
    //         auto dist = terrain_->getHeightMap()->GetMinEdgeDist(Eigen::Vector2d(x,y));
    //         cout << "x: " << x << " y: " << y << " dist: " << dist.transpose() << endl;
    //     }
    // }

    // auto sdf_map = terrain_->getSDF();

    // string filePath = "gradient_map.txt";
    // ofstream writeFile(filePath.data());
    // // test from here
    // if (writeFile.is_open()){
    //     for (double height = 0.14; height< 0.16; height+=0.001){
    //         for (double x_dist = x_min; x_dist < x_max; x_dist+=0.01){
    //             for (double y_dist = y_min; y_dist < y_max; y_dist+=0.1){
    //                 grid_map::Position3 position(x_dist, y_dist, height);
    //                 double cost = sdf_map.getInterpolatedDistanceAt(position);
    //                 grid_map::Vector3 gradient = sdf_map.getDistanceGradientAt(position);

    //                 cout << "==========================================================" << endl;
    //                 cout << "position: " << position.transpose() << endl;
    //                 cout << "cost:" << cost << endl;
    //                 cout << "gradient:" << gradient.transpose() << endl;

    //                 // save file
    //                 writeFile << position.x() << " " << position.y() << " " << position.z() << " " << cost << " " << gradient.x() << " " << gradient.y() << " " << gradient.z() << endl;

    //             }
    //         }
    //     }
    //     // writeFile << ss.str();
    //     writeFile.close();
    // }

    return 0;
}
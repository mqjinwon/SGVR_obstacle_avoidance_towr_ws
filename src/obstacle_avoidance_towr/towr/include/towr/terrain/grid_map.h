#pragma once

#include <towr/terrain/height_map.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_sdf/SignedDistanceField.hpp>

namespace SGVR {

class TowrMap{

public:
    using Ptr = std::shared_ptr<TowrMap>;

    TowrMap();
    TowrMap(towr::HeightMap::TerrainID terrainID,
                const std::vector<std::string>& layers);

    void gridMapInit(
                    const std::string frameId,
                    const grid_map::Length length,
                    const double resolution,
                    const grid_map::Position postition
                    );

    void updateGridMap(const std::string& layer);

    void updateSDF(const std::string& layer, const double heightClearance=0.2);

    void addGridMapLayer(const std::string& layer){
        grid_map_.add(layer);
    }

    // getter & setter
    const grid_map::GridMap getGridMap(){
        return grid_map_;
    }

    const grid_map::SignedDistanceField getSDF(){
        return sdf_;
    }

    const towr::HeightMap::Ptr getHeightMap(){
        return height_map_;
    }

    void setHeightMap(towr::HeightMap::Ptr height_map){
        height_map_ = height_map;
    }

    const towr::HeightMap::TerrainID getTerrainID(){
        return terrainID_;
    }


private:
    towr::HeightMap::Ptr height_map_;
    grid_map::GridMap grid_map_;
    grid_map::SignedDistanceField sdf_;

    towr::HeightMap::TerrainID terrainID_;
    
};

}
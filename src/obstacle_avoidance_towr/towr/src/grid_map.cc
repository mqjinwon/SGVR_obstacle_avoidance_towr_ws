#include <towr/terrain/grid_map.h>

namespace SGVR{

TowrMap::TowrMap(){};

TowrMap::TowrMap(towr::HeightMap::TerrainID terrainID,
                        const std::vector<std::string>& layers):
    terrainID_(terrainID),
    grid_map_(layers)
{
    height_map_ = towr::HeightMap::MakeTerrain(terrainID);
}

void TowrMap::gridMapInit(
                const std::string frameId,
                const grid_map::Length length,
                const double resolution,
                const grid_map::Position postition
                )
{  
    grid_map_.setFrameId(frameId);
    grid_map_.setGeometry(length, resolution, postition);
}

void TowrMap::updateGridMap(const std::string& layer)
{
    for (grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it) {
      grid_map::Position position;
      grid_map_.getPosition(*it, position);
      grid_map_.at(layer, *it) = height_map_->GetHeight(position.x(), position.y());
    }
}

void TowrMap::updateSDF(const std::string& layer, const double heightClearance){
    sdf_.calculateSignedDistanceField(grid_map_, layer, heightClearance);
}

} // namespace SGVR
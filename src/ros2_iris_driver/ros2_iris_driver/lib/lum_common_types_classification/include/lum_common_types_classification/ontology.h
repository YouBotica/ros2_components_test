// Copyright (c)  2023 , Luminar Technologies, Inc.
// This material contains confidential and trade secret information of Luminar
// Technologies. Reproduction, adaptation, and distribution are prohibited,
// except to the extent expressly permitted in writing by Luminar Technologies.


#ifndef LUM_COMMON_TYPES_CLASSIFICATION_ONTOLOGY_H
#define LUM_COMMON_TYPES_CLASSIFICATION_ONTOLOGY_H

#include <cstdint>

namespace lum {
namespace common {
namespace types {
namespace classification {

/// @namespace lum::common::types::classification Classification data types

/// @brief Classification ontology will be represented with 2 bytes, unsigned
using OntologyType = std::uint16_t;

/// @brief Classification confidence will be represented with 4 byte float
using ConfidenceType = float;

// Enum offsets
static constexpr OntologyType UNCLASSIFIED_ONTOLOGY_OFFSET{0U};
static constexpr OntologyType WEATHER_ONTOLOGY_OFFSET{100U};
static constexpr OntologyType DRIVABLE_SURFACE_ONTOLOGY_OFFSET{200U};
static constexpr OntologyType VEGETATION_ONTOLOGY_OFFSET{300U};
static constexpr OntologyType TRAFFIC_BARRIER_ONTOLOGY_OFFSET{1000U};
static constexpr OntologyType LANE_MARKING_ONTOLOGY_OFFSET{6000U};
static constexpr OntologyType VEHICLE_ONTOLOGY_OFFSET{11000U};
static constexpr OntologyType HUMAN_ONTOLOGY_OFFSET{16000U};
static constexpr OntologyType ANIMAL_ONTOLOGY_OFFSET{21000U};
static constexpr OntologyType TRAFFIC_SIGN_ONTOLOGY_OFFSET{26000U};
static constexpr OntologyType TRAFFIC_LIGHT_ONTOLOGY_OFFSET{31000U};

/// @brief Classification Ontology for use across LumPDK
enum class Ontology : OntologyType
{

  // Classifications related to data validity or ability to classify

  UNCLASSIFIED = UNCLASSIFIED_ONTOLOGY_OFFSET + 0U,   ///< No given classification
  NO_RETURN = UNCLASSIFIED_ONTOLOGY_OFFSET + 1U,      ///< No return at this location
  INVALID_RETURN = UNCLASSIFIED_ONTOLOGY_OFFSET + 2U, ///< Point-wise return was not valid
  AMBIGUOUS_RANGE_RETURN =
    UNCLASSIFIED_ONTOLOGY_OFFSET + 3U,     ///< Point-wise return given ambiguous range
  SKY = UNCLASSIFIED_ONTOLOGY_OFFSET + 4U, ///< Point-wise return is sky
  UNKNOWN_OBJECT_ON_DRIVABLE_SURFACE =
    UNCLASSIFIED_ONTOLOGY_OFFSET + 5U, ///< Object on the drivable surface, but not classified
  UNCLASSIFIED_OBJECT_IN_VIEW =
    UNCLASSIFIED_ONTOLOGY_OFFSET + 6U, ///< Object within the field of view, but not classified
  BLOOMED_RETURN = UNCLASSIFIED_ONTOLOGY_OFFSET + 7U, ///< Bloom

  // Classifications related to weather

  FOG_RETURN = WEATHER_ONTOLOGY_OFFSET + 0U,     ///< Point-wise return is fog
  MIST_RETURN = WEATHER_ONTOLOGY_OFFSET + 1U,    ///< Point-wise return is mist
  AEROSOL_RETURN = WEATHER_ONTOLOGY_OFFSET + 2U, ///< Point-wise return is part of an aerosol
  RAIN_RETURN = WEATHER_ONTOLOGY_OFFSET + 3U,    ///< Point-wise return is rain
  SNOW_RETURN = WEATHER_ONTOLOGY_OFFSET + 4U,    ///< Point-wise return is snow
  STEAM_RETURN = WEATHER_ONTOLOGY_OFFSET + 5U,   ///< Point-wise return is steam

  // Classifications related to road detection/tracking

  DIRECTLY_DRIVABLE_SURFACE =
    DRIVABLE_SURFACE_ONTOLOGY_OFFSET + 0U, ///< Point-wise return is part of the drivable road
  INFERRED_ROAD_SURFACE = DRIVABLE_SURFACE_ONTOLOGY_OFFSET +
                          1U, ///< Point is inferred to be a part of the drivable road / free space
  UNDRIVABLE_SURFACE = DRIVABLE_SURFACE_ONTOLOGY_OFFSET + 2U, ///< Point-wise return is not drivable
  SIDEWALK =
    DRIVABLE_SURFACE_ONTOLOGY_OFFSET + 3U, ///< Point-wise return is part of the sidewalk/curb
  PARKING_LOT =
    DRIVABLE_SURFACE_ONTOLOGY_OFFSET + 4U, ///< Point-wise return is part of a parking lot
  ROAD_EDGE_FROM_DIRECTLY_DRIVABLE_SURFACE =
    DRIVABLE_SURFACE_ONTOLOGY_OFFSET + 5U, ///< Road edge derived from drivable surface
  ROAD_EDGE_FROM_INFERRED_ROAD_SURFACE =
    DRIVABLE_SURFACE_ONTOLOGY_OFFSET + 6U, ///< Road edge derived from inferred road surface
  ROAD_EDGE_FROM_UNDRIVABLE_SURFACE =
    DRIVABLE_SURFACE_ONTOLOGY_OFFSET +
    7U, ///< Road edge derived from undrivable surface (e.g. grassy field)
  ROAD_EDGE_FROM_SIDEWALK =
    DRIVABLE_SURFACE_ONTOLOGY_OFFSET + 8U, ///< Road edge derived from sidewalk
  ROAD_EDGE_FROM_PARKING_LOT =
    DRIVABLE_SURFACE_ONTOLOGY_OFFSET + 9U, ///< Road edge derived from parking lot
  ROAD_EDGE_FROM_TRAFFIC_BARRIER =
    DRIVABLE_SURFACE_ONTOLOGY_OFFSET + 10U, ///< Road edge derived from generic traffic barrier
  ROAD_EDGE_FROM_VERTICAL_VEGETATION =
    DRIVABLE_SURFACE_ONTOLOGY_OFFSET + 11U, ///< Road edge derived from vertical vegetation
  SHOULDER_PAVED = DRIVABLE_SURFACE_ONTOLOGY_OFFSET + 12U,    ///< Shoulder paved
  SHOULDER_GRASS = DRIVABLE_SURFACE_ONTOLOGY_OFFSET + 13U,    ///< Shoulder grass
  SHOULDER_GRAVEL = DRIVABLE_SURFACE_ONTOLOGY_OFFSET + 14U,   ///< Shoulder gravel
  BIKE_LANE_SURFACE = DRIVABLE_SURFACE_ONTOLOGY_OFFSET + 15U, ///< Bike lane surface

  // Classifications related to vegetation

  VERTICAL_VEGETATION =
    VEGETATION_ONTOLOGY_OFFSET + 0U, ///< Vertical vegetation, e.g. trees and bushes

  // Classifications related to traffic barriers

  UNCLASSIFIED_TRAFFIC_BARRIER =
    TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 0U, ///< Classification is known to be traffic barrier, but
                                          ///< specific type of traffic barrier is unknown
  ROAD_MEDIAN = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 1U, ///< Median separating parts of a road, often
                                                      ///< separating directions of traffic
  CONSTRUCTION_CONE = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 2U,    ///< Traffic/construction cone
  CONSTRUCTION_PYLON = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 3U,   ///< Traffic/construction pylon
  CONSTRUCTION_BARRIER = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 4U, ///< Generic construction barrier
  BUILDING = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 5U,             ///< Building
  BRIDGE = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 6U,               ///< Bridge
  TUNNEL = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 7U,               ///< Tunnel
  FENCE = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 8U,                ///< Fence
  WALL = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 9U,                 ///< Wall
  JERSEY_BARRIER = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 10U,      ///< Barrier of type Jersey
  HORIZONTAL_OBSTRUCTION =
    TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 11U,            ///< obstruction demarcating specific regions
  GUARD_RAIL = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 12U, ///< Rail demarctating a zone
  TIRE = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 13U,       ///< Tire
  FIRE_HYDRANT = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 14U,       ///< Fire hydrant
  GARBAGE_CAN = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 15U,        ///< Can holds garbage
  BOX = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 16U,                ///< Generic bag
  DEBRIS = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 17U,             ///< Generic debris
  POLE = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 18U,               ///< Generic pole
  TRAFFIC_SIGN_POLE = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 19U,  ///< Pole holding up a traffic sign
  TRAFFIC_LIGHT_POLE = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 20U, ///< Pole holding up a traffic light
  CRASH_BARRIER = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 21U,      ///< crash barrier
  TRIANGLE_SIGNAL =
    TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 22U,      ///< triangle signal safety marker put on road
  GATE = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 23U, ///< openable fence
  RAILROAD_CROSSING =
    TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 24U,               ///< specific gate for railroad crossings
  LOW_CLEARANCE = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 25U, ///< low clearance
  CONSTRUCTION_BARREL = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 26U,    ///< construction barrel
  CONSTRUCTION_BARRICADE = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 27U, ///< construction barricade
  DEBRIS_BRICK = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 28U,           ///< brick, debris
  DEBRIS_BOTTLE_CRATE = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 29U,    ///< bottle crate, debris
  DEBRIS_RIM = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 30U,             ///< rim, debris
  DEBRIS_BICYCLE = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 31U,         ///< bicycle, debris
  DEBRIS_PALLET = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 32U,          ///< pallet, debris
  DEBRIS_ALUMINUM_BLOCK = TRAFFIC_BARRIER_ONTOLOGY_OFFSET + 33U,  ///< aluminum_block, debris

  // Classifications related to lane detection/tracking

  UNCLASSIFIED_LANE_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 0U, ///< Classification is known to be lane marking, but specific
                                       ///< lane marking type is unknown
  SINGLE_DASHED_LANE_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET +
    1U, ///< Lane marking, member of single dashed lane, lane color unknown
  SINGLE_SOLID_LANE_MARKING = LANE_MARKING_ONTOLOGY_OFFSET +
                              2U, ///< Lane marking, member of single solid lane, lane color unknown
  SINGLE_DOTTED_LANE_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET +
    3U, ///< Lane marking, member of single dotted lane, lane color unknown
  DOUBLE_DASHED_LANE_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET +
    4U, ///< Lane marking, member of double dashed lane, lane color unknown
  DOUBLE_SOLID_LANE_MARKING = LANE_MARKING_ONTOLOGY_OFFSET +
                              5U, ///< Lane marking, member of double solid lane, lane color unknown
  DOUBLE_DOTTED_LANE_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET +
    6U, ///< Lane marking, member of double dotted lane, lane color unknown
  SINGLE_YELLOW_DASHED_LANE_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 7U, ///< Lane marking, member of single yellow dashed lane
  SINGLE_YELLOW_SOLID_LANE_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 8U, ///< Lane marking, member of single yellow solid lane
  SINGLE_YELLOW_DOTTED_LANE_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 9U, ///< Lane marking, member of single yellow dotted lane
  SINGLE_WHITE_DASHED_LANE_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 10U, ///< Lane marking, member of single white dashed lane
  SINGLE_WHITE_SOLID_LANE_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 11U, ///< Lane marking, member of single white solid lane
  SINGLE_WHITE_DOTTED_LANE_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 12U, ///< Lane marking, member of single white dotted lane
  DOUBLE_YELLOW_DASHED_LANE_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 13U, ///< Lane marking, member of double yellow dashed lane
  DOUBLE_YELLOW_SOLID_LANE_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 14U, ///< Lane marking, member of double yellow solid lane
  DOUBLE_YELLOW_DOTTED_LANE_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 15U, ///< Lane marking, member of double yellow dotted lane
  DOUBLE_WHITE_DASHED_LANE_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 16U, ///< Lane marking, member of double white dashed lane
  DOUBLE_WHITE_SOLID_LANE_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 17U, ///< Lane marking, member of double white solid lane
  DOUBLE_WHITE_DOTTED_LANE_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 18U, ///< Lane marking, member of double white dotted lane
  CROSSWALK_LANE_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 19U, ///< Lane marking, member of pedestrian crosswalk
  SOLID_WHITE_STOP_LINE_LANE_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 20U, ///< Solid white lane marking designating a traffic stop
  SOLID_WHITE_BIKE_LANE_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 21U, ///< Solid white lane marking to designate a bike lane
  STRAIGHT_LANE_ROAD_PAINT_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET +
    22U, ///< Road paint designating a lane where traffic continues straight
  LEFT_TURN_ONLY_ROAD_PAINT_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET +
    23U, ///< Road paint designating a lane where traffic must turn left
  STRAIGHT_LEFT_TURN_LANE_ROAD_PAINT_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 24U, ///< Road paint designating a lane where traffic must turn
                                        ///< left or continue straight
  RIGHT_TURN_ONLY_ROAD_PAINT_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET +
    25U, ///< Road paint designating a lane where traffic must turn right
  STRAIGHT_RIGHT_TURN_LANE_ROAD_PAINT_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 26U, ///< Road paint designating a lane where traffic must turn
                                        ///< left or continue straight
  HIGH_OCCUPANCY_VEHICLE_LANE_ROAD_PAINT_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 27U, ///< Road paint designating a lane restricted for a driver
                                        ///< with one or more passengers
  MERGE_LEFT_ROAD_PAINT_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET +
    28U, ///< Road paint designating a lane where traffic must merge to the left
  MERGE_RIGHT_ROAD_PAINT_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET +
    29U, ///< Road paint designating a lane where traffic must merge to the right
  EXIT_LANE_ROAD_PAINT_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 30U, ///< Road paint designating a lane where traffic must exit
  STOP_WRITTEN_ROAD_PAINT_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 31U, ///< Road paint where stop is written
  KEEP_CLEAR_WRITTEN_ROAD_PAINT_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 32U, ///< Road paint where keep clear is written
  UNCLASSIFIED_WRITTEN_ROAD_PAINT_MARKING =
    LANE_MARKING_ONTOLOGY_OFFSET + 33U,                                  ///< Road paint other
  AMBIGUOUS_PATTERN_LANE_MARKING = LANE_MARKING_ONTOLOGY_OFFSET + 34U,   ///< ambiguous_pattern
  GENERIC_WHITE_SOLID_LANE_MARKING = LANE_MARKING_ONTOLOGY_OFFSET + 35U, ///< white_solid_other

  // Classifications related to vehicles

  VEHICLE = VEHICLE_ONTOLOGY_OFFSET + 0U,       ///< Classification is known to be vehicle, but
                                                ///< specific vehicle type/stats is unknown
  LARGE_VEHICLE = VEHICLE_ONTOLOGY_OFFSET + 1U, ///< Large vehicle, e.g. truck, bus
  LARGE_VEHICLE_TRAILER = VEHICLE_ONTOLOGY_OFFSET + 2U,    ///< Trailer attached to large vehicle
  TRAIN = VEHICLE_ONTOLOGY_OFFSET + 3U,                    ///< Train
  MOTORCYCLE = VEHICLE_ONTOLOGY_OFFSET + 4U,               ///< Motorcycle, with or without rider
  BICYCLE = VEHICLE_ONTOLOGY_OFFSET + 5U,                  ///< Bicycle, with or without rider
  EMERGENCY_VEHICLE_ACTIVE = VEHICLE_ONTOLOGY_OFFSET + 6U, ///< Emergency vehicle with active lights
  EMERGENCY_VEHICLE_INACTIVE =
    VEHICLE_ONTOLOGY_OFFSET + 7U,                       ///< Emergency vehicle wihtout active lights
  POLICE_VEHICLE_ACTIVE = VEHICLE_ONTOLOGY_OFFSET + 8U, ///< Police vehicle with active lights
  POLICE_VEHICLE_INACTIVE = VEHICLE_ONTOLOGY_OFFSET + 9U, ///< Police vehicle wihtout active lights
  CONSTRUCTION_VEHICLE = VEHICLE_ONTOLOGY_OFFSET + 10U,   ///< Generic construction vehicle
  BOAT = VEHICLE_ONTOLOGY_OFFSET + 11U,                   ///< Boat, in water or on land
  BUS = VEHICLE_ONTOLOGY_OFFSET + 12U,                    ///< Bus
  SCHOOL_BUS = VEHICLE_ONTOLOGY_OFFSET + 13U,             ///< School Bus
  EMERGENCY_VEHICLE_SEDAN_SUV_ACTIVE =
    VEHICLE_ONTOLOGY_OFFSET + 14U, ///< emergency vehicle sedan with active lights
  EMERGENCY_VEHICLE_SEDAN_SUV_INACTIVE =
    VEHICLE_ONTOLOGY_OFFSET + 15U, ///< emergency vehicle sedan without active lights
  EMERGENCY_VEHICLE_AMBULANCE_ACTIVE =
    VEHICLE_ONTOLOGY_OFFSET + 16U, ///< emergency vehicle ambulance with active lights
  EMERGENCY_VEHICLE_AMBULANCE_INACTIVE =
    VEHICLE_ONTOLOGY_OFFSET + 17U, ///< emergency vehicle ambulance without active lights
  EMERGENCY_VEHICLE_FIRE_TRUCK_ACTIVE =
    VEHICLE_ONTOLOGY_OFFSET + 18U, ///< emergency vehicle fire_truck with active lights
  EMERGENCY_VEHICLE_FIRE_TRUCK_INACTIVE =
    VEHICLE_ONTOLOGY_OFFSET + 19U, ///< emergency vehicle fire_truck without active lights
  POLICE_MOTORCYCLE_ACTIVE =
    VEHICLE_ONTOLOGY_OFFSET + 20U, ///< Police motorcycle with active lights
  POLICE_MOTORCYCLE_INACTIVE =
    VEHICLE_ONTOLOGY_OFFSET + 21U, ///< Police motorcycle wihtout active lights
  CONSTRUCTION_VEHICLE_WHEELED =
    VEHICLE_ONTOLOGY_OFFSET + 22U, ///< Construction vehicles with wheels
  VEHICLE_APPENDAGE_ROOF_MOUNTED =
    VEHICLE_ONTOLOGY_OFFSET + 23U, ///< object attached tot he outside of a vehicle on the roof
  VEHICLE_APPENDAGE_REAR_MOUNTED =
    VEHICLE_ONTOLOGY_OFFSET + 24U, ///< object attached tot he outside of a vehicle on the rear
  VEHICLE_VAN = VEHICLE_ONTOLOGY_OFFSET + 25U,   ///< van for very specific classification
  VEHICLE_TRUCK = VEHICLE_ONTOLOGY_OFFSET + 26U, ///< truck for very specific classification
  VEHICLE_SEDAN = VEHICLE_ONTOLOGY_OFFSET + 27U, ///< sedan for very specific classification

  // Classifications related to humans

  HUMAN = HUMAN_ONTOLOGY_OFFSET + 0U,      ///< Known to be a human, but subcategory unknown
  PEDESTRIAN = HUMAN_ONTOLOGY_OFFSET + 1U, ///< Human pedestrian
  MOTORIZED_PEDESTRIAN =
    HUMAN_ONTOLOGY_OFFSET + 2U, ///< Motorized human pedestrian, e.g. pedestrian on scooter
  PEDESTRIAN_APPENDAGE =
    HUMAN_ONTOLOGY_OFFSET +
    3U, ///< A unique object a pedestrian may be carrying, pushing, wearing, etc
  BICYCLIST = HUMAN_ONTOLOGY_OFFSET + 4U,                    ///< human on a bicycle
  MOTORCYCLIST = HUMAN_ONTOLOGY_OFFSET + 5U,                 ///< human on a motorcycle
  PEDESTRIAN_DIRECTING_TRAFFIC = HUMAN_ONTOLOGY_OFFSET + 6U, ///< crossing gaurd
  PEDESTRIAN_LYING_CHILD = HUMAN_ONTOLOGY_OFFSET + 7U,       ///< child lying flat
  PEDESTRIAN_LYING_ADULT = HUMAN_ONTOLOGY_OFFSET + 8U,       ///< adult lying flat

  // Classifications related to animals

  UNCLASSIFIED_ANIMAL = ANIMAL_ONTOLOGY_OFFSET + 0U, ///< Classification is known to be animal, but
                                                     ///< specific type of animal is unclassified
  ANIMAL_BIRD = ANIMAL_ONTOLOGY_OFFSET + 1U,         ///< Bird
  ANIMAL_DOG = ANIMAL_ONTOLOGY_OFFSET + 2U,          ///< Dog
  ANIMAL_CAT = ANIMAL_ONTOLOGY_OFFSET + 3U,          ///< Cat
  ANIMAL_DEER = ANIMAL_ONTOLOGY_OFFSET + 4U,         ///< Deer
  ANIMAL_LIVESTOCK = ANIMAL_ONTOLOGY_OFFSET + 5U,    ///< Livestock
  SMALL_ANIMAL = ANIMAL_ONTOLOGY_OFFSET + 6U,        ///< small animal
  LARGE_ANIMAL = ANIMAL_ONTOLOGY_OFFSET + 7U,        ///< large animal

  // Classifications related to traffic signs

  UNCLASSIFIED_TRAFFIC_SIGN =
    TRAFFIC_SIGN_ONTOLOGY_OFFSET + 0U, ///< Classification is known to be traffic sign, but specific
                                       ///< type of traffic sign is unclassified
  UNCLASSIFIED_OVERHEAD_TRAFFIC_SIGN =
    TRAFFIC_SIGN_ONTOLOGY_OFFSET + 1U, ///< Classification is known to be overhead traffic sign, but
                                       ///< specific type of overhead traffic sign is unclassified
  UNCLASSIFIED_ATTACHED_TRAFFIC_SIGN =
    TRAFFIC_SIGN_ONTOLOGY_OFFSET + 2U, ///< Classification is known to be attached traffic sign (ie
                                       ///< attached to gaurd rail), but specific type of attached
                                       ///< traffic sign is unclassified
  STOP_SIGN = TRAFFIC_SIGN_ONTOLOGY_OFFSET + 3U,            ///< Stop sign
  STOP_AHEAD_SIGN = TRAFFIC_SIGN_ONTOLOGY_OFFSET + 4U,      ///< Stop ahead sign
  YIELD_SIGN = TRAFFIC_SIGN_ONTOLOGY_OFFSET + 5U,           ///< Yield sign
  YIELD_AHEAD_SIGN = TRAFFIC_SIGN_ONTOLOGY_OFFSET + 6U,     ///< Yield ahead sign
  WRONG_WAY_SIGN = TRAFFIC_SIGN_ONTOLOGY_OFFSET + 7U,       ///< Wrong way sign
  ONE_WAY_SIGN = TRAFFIC_SIGN_ONTOLOGY_OFFSET + 8U,         ///< One way sign
  TWO_WAY_TRAFFIC_SIGN = TRAFFIC_SIGN_ONTOLOGY_OFFSET + 9U, ///< Two way sign
  DO_NOT_ENTER_SIGN = TRAFFIC_SIGN_ONTOLOGY_OFFSET + 10U,   ///< Do not enter sign
  NO_TURN_SIGN = TRAFFIC_SIGN_ONTOLOGY_OFFSET + 11U,        ///< No turn sign
  NO_UTURN_SIGN = TRAFFIC_SIGN_ONTOLOGY_OFFSET + 12U,       ///< No uturn sign
  SPEED_LIMIT_SIGN = TRAFFIC_SIGN_ONTOLOGY_OFFSET + 13U,    ///< speed limit sign
  PEDESTRIAN_CROSSING_SIGN =
    TRAFFIC_SIGN_ONTOLOGY_OFFSET + 14U, ///< Sign designating a pedestrian crossing
  CONSTRUCTION_TEXT_SIGN = TRAFFIC_SIGN_ONTOLOGY_OFFSET + 15U, ///< Sign related to construction
  SPEED_LIMIT_SIGN_CIRCULAR =
    TRAFFIC_SIGN_ONTOLOGY_OFFSET + 16U, ///< speed limit sign thats circular

  // Classifications related to traffic lights

  UNCLASSIFIED_TRAFFIC_LIGHT =
    TRAFFIC_LIGHT_ONTOLOGY_OFFSET + 0U, ///< Classification is known to be traffic light, but
                                        ///< traffic light state/type is unknown
  TRAFFIC_LIGHT_RED_ACTIVE =
    TRAFFIC_LIGHT_ONTOLOGY_OFFSET + 1U, ///< Traffic light with red light active
  TRAFFIC_LIGHT_YELLOW_ACTIVE =
    TRAFFIC_LIGHT_ONTOLOGY_OFFSET + 2U, ///< Traffic light with yellow light active
  TRAFFIC_LIGHT_GREEN_ACTIVE =
    TRAFFIC_LIGHT_ONTOLOGY_OFFSET + 3U, ///< Traffic light with green light active
  TRAFFIC_LIGHT_RED_ARROW_ACTIVE =
    TRAFFIC_LIGHT_ONTOLOGY_OFFSET + 4U, ///< Traffic light with red arrow active
  TRAFFIC_LIGHT_YELLOW_ARROW_ACTIVE =
    TRAFFIC_LIGHT_ONTOLOGY_OFFSET + 5U, ///< Traffic light with yellow arrow active
  TRAFFIC_LIGHT_GREEN_ARROW_ACTIVE =
    TRAFFIC_LIGHT_ONTOLOGY_OFFSET + 6U, ///< Traffic light with green arrow active
  TRAFFIC_LIGHT_FLASHING_RED_ACTIVE =
    TRAFFIC_LIGHT_ONTOLOGY_OFFSET + 7U, ///< Traffic light with flashing red active
  TRAFFIC_LIGHT_FLASHING_YELLOW_ACTIVE =
    TRAFFIC_LIGHT_ONTOLOGY_OFFSET + 8U, ///< Traffic light with flashing yellow active
  TRAFFIC_LIGHT_FLASHING_GREEN_ACTIVE =
    TRAFFIC_LIGHT_ONTOLOGY_OFFSET + 9U, ///< Traffic light with flashing green active
  TRAFFIC_LIGHT_FLASHING_RED_ARROW_ACTIVE =
    TRAFFIC_LIGHT_ONTOLOGY_OFFSET + 10U, ///< Traffic light with flashing red arrow active
  TRAFFIC_LIGHT_FLASHING_YELLOW_ARROW_ACTIVE =
    TRAFFIC_LIGHT_ONTOLOGY_OFFSET + 11U, ///< Traffic light with flashing yellow arrow active
  TRAFFIC_LIGHT_FLASHING_GREEN_ARROW_ACTIVE =
    TRAFFIC_LIGHT_ONTOLOGY_OFFSET + 12U ///< Traffic light with flashing green arrow active

};

/// @brief Structure for storing a classification and its confidence
struct ClassificationWithConfidence
{
  OntologyType classification{OntologyType{0U}}; ///< Classification from ontology
  ConfidenceType confidence{ConfidenceType{0U}}; ///< Confidence of class: 0 - 1
};

} // namespace classification
} // namespace types
} // namespace common
} // namespace lum

#endif // LUM_COMMON_TYPES_CLASSIFICATION_ONTOLOGY_H

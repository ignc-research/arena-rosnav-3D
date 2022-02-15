/**
 * light_scan_sim wall_segment_sim.cpp
 * @brief Cast simulated laser ray at vector wall segments
 *
 * @copyright 2017 Joseph Duchesne
 * @author Joseph Duchesne
 */

#include "light_scan_sim/wall_segment_sim.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <light_scan_sim/Material.h>
#include <utility>

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

/**
 * Construct the WallSegmentSim
 * 
 * @param segments  The list of segments making up the world
 * @param materials The list of materials making up the world
 */
WallSegmentSim::WallSegmentSim(light_scan_sim::SegmentList segments, light_scan_sim::MaterialList materials) {
  segments_ = segments;
  materials_ = materials;

  // Create physics world
  this->InitializeWorld();
};

/**
 * @brief Trace a point across the wall segments
 *
 * @param start  The scan trace start point
 * @param theta  The scan trace angle
 * @param length The scan max length (in pixels)
 * @param ray_max The range used beyond which is 'no return'
 * @param range  The return distance to point of contact
 *
 * @return true if hit, false otherwise
 */
bool WallSegmentSim::Trace(double x, double y, double theta, double length, double ray_max, double &range) {
  static std::default_random_engine random;
  static std::uniform_real_distribution<> uniform_dist(0, 1);

  if (std::isnan(theta)) {
    return false;
  }

  //set up input
  b2RayCastInput input;
  input.p1 = b2Vec2(x, y);
  input.p2 = input.p1 + length*b2Vec2(cos(theta), sin(theta));
  input.maxFraction = 1;


  // Todo: Create list of all collided lines, sort and process in order
  
  //check every fixture of every body to find closest
  typedef std::pair<b2RayCastOutput, light_scan_sim::Material*> HitPair;
  std::vector<HitPair> hits;

  for (b2Body* body = world_->GetBodyList(); body; body = body->GetNext()) {
    for (b2Fixture* fixture = body->GetFixtureList(); fixture; fixture = fixture->GetNext()) {
      b2RayCastOutput output;
      light_scan_sim::Material* material = (light_scan_sim::Material*)(body->GetUserData());
      // ROS_INFO_STREAM("fixture: " << segment->start[0] << ", " << segment->start[1]);
      if ( ! fixture->RayCast( &output, input, 0) ) {
        continue;
      }
      if ( output.fraction >= 0 && output.fraction <= 1.0 ) {
        hits.push_back(HitPair(output, material));
      }
    }
  }

  // Sort from closest to farthest (fraction ascending)
  std::sort(hits.begin(), hits.end(),
            [](const HitPair &a, const HitPair &b){ return a.first.fraction < b.first.fraction; });
  
  // If there are any hits, process them from closest to most distant
  if (hits.size() > 0) {
    for (auto hit : hits) {
      double hit_chance = hit.second->max_return;

      if (hit.second->angular_return != 0) {
        b2Vec2 in_vec = input.p2 - input.p1;
        in_vec.Normalize();
        double angle_of_incidence = fabs(acos(b2Dot(in_vec, hit.first.normal)));
        if (angle_of_incidence > M_PI_2) {
          angle_of_incidence = M_PI - angle_of_incidence;  // handle hitting either side of the line the same
        }
        hit_chance += hit.second->angular_return * angle_of_incidence* 180.0/M_PI;  // Convert 
        hit_chance = constrain(hit_chance, hit.second->min_return, hit.second->max_return);
      }

      if (uniform_dist(random) < hit_chance) {  // If the light hit
        // ROS_INFO_STREAM("Closest: " << range);
        range = hit.first.fraction * length;
        return true;
      } else {  // We didn't 'hit', pass through if transparent or no return if opaque
        if (hit.second->type == "opaque") {
          range = ray_max + 1.0;  // Denotes 'no return'
          return true;
        }
      }
    }
  }

  return false;  // No collision, defer to occupancy grid collision
}


/**
 * @brief Initialize the physics world
 */
void WallSegmentSim::InitializeWorld() {
  world_ = std::make_shared<b2World>(b2Vec2(0,0));

  for (auto i : segments_.segments) {
    // ROS_INFO_STREAM("Initializing segment of type " << materials_.materials[i.type].name);

    // Create shape representing segment
    b2EdgeShape edge_shape;
    edge_shape.Set( b2Vec2(i.start[0], i.start[1]), b2Vec2(i.end[0], i.end[1]) );
   
    // Create fixture definition of the shape
    b2FixtureDef fixture_def; 
    fixture_def.shape = &edge_shape;
  
    // Create body to contain fixture
    b2BodyDef body_def; 
    body_def.type = b2_staticBody; // can't move
    body_def.position.Set(0, 0); // origin

    b2Body* segment_body = world_->CreateBody(&body_def);
    segment_body->CreateFixture(&fixture_def);  // Attach fixture to body
    segment_body->SetUserData(&materials_.materials[i.type]);  // Reference this segment as user data
  }

  ROS_ERROR("Initialize world completed");
}

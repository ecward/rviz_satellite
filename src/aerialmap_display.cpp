/*
 * AerialMapDisplay.cpp
 *
 *  Copyright (c) 2014 Gaeth Cross. Apache 2 License.
 *
 *  This file is part of rviz_satellite.
 *
 *	Created on: 07/09/2014
 */

#include <QtGlobal>
#include <QImage>
#include <QPainter>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreImageCodec.h>
#include <OGRE/OgreVector3.h>

#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/grid.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/tf_frame_property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/validate_floats.h"
#include "rviz/display_context.h"

#include "aerialmap_display.h"

#define FRAME_CONVENTION_XYZ_ENU (0)  //  X -> East, Y -> North
#define FRAME_CONVENTION_XYZ_NED (1)  //  X -> North, Y -> East
#define FRAME_CONVENTION_XYZ_NWU (2)  //  X -> North, Y -> West

// Max number of adjacent blocks to support.
static constexpr int kMaxBlocks = 8;
// Max zoom level to support.
static constexpr int kMaxZoom = 22;

// TODO(gareth): If higher zooms are ever supported, change calculations from
// int to long wherever applicable.
static_assert((1 << kMaxZoom) < std::numeric_limits<unsigned int>::max(), "");

Ogre::TexturePtr textureFromImage(const QImage &image,
                                  const std::string &name) {
  //  convert to 24bit rgb
  QImage converted = image.convertToFormat(QImage::Format_RGB888).mirrored();

  //  create texture
  Ogre::TexturePtr texture;
  Ogre::DataStreamPtr data_stream;
  data_stream.bind(new Ogre::MemoryDataStream((void *)converted.constBits(),
                                              converted.byteCount()));

  const Ogre::String res_group =
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;
  Ogre::TextureManager &texture_manager = Ogre::TextureManager::getSingleton();
  //  swap byte order when going from QImage to Ogre
  texture = texture_manager.loadRawData(name, res_group, data_stream,
                                        converted.width(), converted.height(),
                                        Ogre::PF_B8G8R8, Ogre::TEX_TYPE_2D, 0);
  return texture;
}

namespace rviz {

AerialMapDisplay::AerialMapDisplay()
    : Display(), map_id_(0), scene_id_(0), dirty_(false),
      received_msg_(false) {

  static unsigned int map_ids = 0;
  map_id_ = map_ids++; //  global counter of map ids

  topic_property_ = new RosTopicProperty(
      "Topic", "", QString::fromStdString(
                       ros::message_traits::datatype<sensor_msgs::NavSatFix>()),
      "nav_msgs::Odometry topic to subscribe to.", this, SLOT(updateTopic()));

  frame_property_ = new TfFrameProperty("Robot frame", "world",
                                        "TF frame for the moving robot.", this,
                                        nullptr, false, SLOT(updateFrame()), this);

  dynamic_reload_property_ =
      new Property("Dynamically reload", true,
                   "Reload as robot moves. Frame option must be set.",
                   this, SLOT(updateDynamicReload()));

  alpha_property_ = new FloatProperty(
      "Alpha", 0.7, "Amount of transparency to apply to the map.", this,
      SLOT(updateAlpha()));
  alpha_ = alpha_property_->getValue().toFloat();
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);
  alpha_property_->setShouldBeSaved(true);

  draw_under_property_ =
      new Property("Draw Behind", false,
                   "Rendering option, controls whether or not the map is always"
                   " drawn behind everything else.",
                   this, SLOT(updateDrawUnder()));
  draw_under_property_->setShouldBeSaved(true);
  draw_under_ = draw_under_property_->getValue().toBool();

  //  output, resolution of the map in meters/pixel
  resolution_property_ = new FloatProperty(
      "Resolution", 0, "Resolution of the map. (Read only)", this);
  resolution_property_->setReadOnly(true);

  //  properties for map
  object_uri_property_ = new StringProperty(
      "Object URI", "http://otile1.mqcdn.com/tiles/1.0.0/sat/{z}/{x}/{y}.jpg",
      "URL from which to retrieve map tiles.", this, SLOT(updateObjectURI()));
  object_uri_property_->setShouldBeSaved(true);
  object_uri_ = object_uri_property_->getStdString();

  const QString zoom_desc = QString::fromStdString(
      "Zoom level (0 - " + std::to_string(kMaxZoom) + ")");
  zoom_property_ =
      new IntProperty("Zoom", 16, zoom_desc, this, SLOT(updateZoom()));
  zoom_property_->setShouldBeSaved(true);
  zoom_property_->setMin(0);
  zoom_property_->setMax(kMaxZoom);
  zoom_ = zoom_property_->getInt();

  const QString blocks_desc = QString::fromStdString(
      "Adjacent blocks (0 - " + std::to_string(kMaxBlocks) + ")");
  blocks_property_ =
      new IntProperty("Blocks", 3, blocks_desc, this, SLOT(updateBlocks()));
  blocks_property_->setShouldBeSaved(true);
  blocks_property_->setMin(0);
  blocks_property_->setMax(kMaxBlocks);

  frame_convention_property_ =
      new EnumProperty("Frame Convention", "XYZ -> ENU",
                       "Convention for mapping cartesian frame to the compass",
                       this, SLOT(updateFrameConvention()));
  frame_convention_property_->addOptionStd("XYZ -> ENU",
                                           FRAME_CONVENTION_XYZ_ENU);
  frame_convention_property_->addOptionStd("XYZ -> NED",
                                           FRAME_CONVENTION_XYZ_NED);
  frame_convention_property_->addOptionStd("XYZ -> NWU",
                                           FRAME_CONVENTION_XYZ_NWU);

  //  updating one triggers reload
  updateBlocks();
}

AerialMapDisplay::~AerialMapDisplay() {
  unsubscribe();
  clear();
}

void AerialMapDisplay::onInitialize() {
  frame_property_->setFrameManager(context_->getFrameManager());
}

void AerialMapDisplay::onEnable() { subscribe(); }

void AerialMapDisplay::onDisable() {
  unsubscribe();
  clear();
}

void AerialMapDisplay::subscribe() {
  if (!isEnabled()) {
    return;
  }

  if (!topic_property_->getTopic().isEmpty()) {
    try {
      ROS_INFO("Subscribing to %s", topic_property_->getTopicStd().c_str());
      coord_sub_ =
          update_nh_.subscribe(topic_property_->getTopicStd(), 1,
                               &AerialMapDisplay::navFixCallback, this);

      setStatus(StatusProperty::Ok, "Topic", "OK");
    }
    catch (ros::Exception &e) {
      setStatus(StatusProperty::Error, "Topic",
                QString("Error subscribing: ") + e.what());
    }
  }
}

void AerialMapDisplay::unsubscribe() {
  coord_sub_.shutdown();
  ROS_INFO("Unsubscribing.");
}

void AerialMapDisplay::updateDynamicReload() {
  // nothing to do here, when robot GPS updates the tiles will reload
}

void AerialMapDisplay::updateAlpha() {
  alpha_ = alpha_property_->getFloat();
  dirty_ = true;
  ROS_INFO("Changing alpha to %f", alpha_);
}

void AerialMapDisplay::updateFrame() {
  ROS_INFO_STREAM("Changing robot frame to " << frame_property_->getFrameStd());
  transformAerialMap();
}

void AerialMapDisplay::updateDrawUnder() {
  /// @todo: figure out why this property only applies to some objects
  draw_under_ = draw_under_property_->getValue().toBool();
  dirty_ = true; //  force update
  ROS_INFO("Changing draw_under to %s", ((draw_under_) ? "true" : "false"));
}

void AerialMapDisplay::updateObjectURI() {
  object_uri_ = object_uri_property_->getStdString();
  loadImagery(); //  reload all imagery
}

void AerialMapDisplay::updateZoom() {
  const int zoom = std::max(0, std::min(kMaxZoom, zoom_property_->getInt()));
  if (zoom != zoom_) {
    zoom_ = zoom;
    loadImagery();
  }
}

void AerialMapDisplay::updateBlocks() {
  const int blocks =
      std::max(0, std::min(kMaxBlocks, blocks_property_->getInt()));
  if (blocks != blocks_) {
    blocks_ = blocks;
    loadImagery();
  }
}

void AerialMapDisplay::updateFrameConvention() {
  transformAerialMap();
}

void AerialMapDisplay::updateTopic() {
  unsubscribe();
  clear();
  subscribe();
}

void AerialMapDisplay::clear() {
  setStatus(StatusProperty::Warn, "Message", "No map received");
  clearGeometry();
  //  the user has cleared here
  received_msg_ = false;
  //  cancel current imagery, if any
  loader_.reset();
}

void AerialMapDisplay::clearGeometry() {
  for (MapObject &obj : objects_) {
    //  destroy object
    scene_node_->detachObject(obj.object);
    scene_manager_->destroyManualObject(obj.object);
    //  destroy texture
    if (!obj.texture.isNull()) {
      Ogre::TextureManager::getSingleton().remove(obj.texture->getName());
    }
    //  destroy material
    if (!obj.material.isNull()) {
      Ogre::MaterialManager::getSingleton().remove(obj.material->getName());
    }
  }
  objects_.clear();
}

void AerialMapDisplay::update(float, float) {
    //update is called very frequently
  //  creates all geometry, if necessary
  assembleScene();
  //  draw
  context_->queueRender();
}

void
AerialMapDisplay::navFixCallback(const sensor_msgs::NavSatFixConstPtr &msg) {
    //only accept nav_sat data that we can transform!
    tf::TransformListener * tf_listen = context_->getFrameManager()->getTFClient();
    //I don't know why this fails like 9/10 times...
    if(tf_listen->canTransform(frame_property_->getFrameStd(),
                               msg->header.frame_id,
                               msg->header.stamp))
    {

        // If the new (lat,lon) falls into a different tile then we have some
        // reloading to do.
        if (!received_msg_ ||
                (loader_ && !loader_->insideCentreTile(msg->latitude, msg->longitude) &&
                 dynamic_reload_property_->getValue().toBool())) {
            ref_fix_ = *msg;
            ROS_INFO("Reference point set to: %.12f, %.12f", ref_fix_.latitude,
                     ref_fix_.longitude);
            setStatus(StatusProperty::Warn, "Message", "Loading map tiles.");

            //  re-load imagery
            received_msg_ = true;
            loadImagery();
            transformAerialMap();
        }
    } else {
        ROS_DEBUG_STREAM("Can't transform nav_sat data in frame " << msg->header.frame_id << " to " <<
                        frame_property_->getFrameStd() << " ignoring!");
    }
}

void AerialMapDisplay::loadImagery() {
  //  cancel current imagery, if any
  loader_.reset();
  
  if (!received_msg_) {
    //  no message received from publisher
    return;
  }
  if (object_uri_.empty()) {
    setStatus(StatusProperty::Error, "Message",
              "Received message but object URI is not set");
  }

  try {
    loader_.reset(new TileLoader(object_uri_, ref_fix_.latitude,
                                 ref_fix_.longitude, zoom_, blocks_, this));
  } catch (std::exception &e) {
    setStatus(StatusProperty::Error, "Message", QString(e.what()));
    return;
  }

  QObject::connect(loader_.get(), SIGNAL(errorOcurred(QString)), this,
                   SLOT(errorOcurred(QString)));
  QObject::connect(loader_.get(), SIGNAL(finishedLoading()), this,
                   SLOT(finishedLoading()));
  QObject::connect(loader_.get(), SIGNAL(initiatedRequest(QNetworkRequest)), this,
                   SLOT(initiatedRequest(QNetworkRequest)));
  QObject::connect(loader_.get(), SIGNAL(receivedImage(QNetworkRequest)), this,
                   SLOT(receivedImage(QNetworkRequest)));
  //  start loading images
  loader_->start();
}

void AerialMapDisplay::assembleScene() {
  if (!dirty_) {
    return; //  nothing to update
  }
  dirty_ = false;
  
  if (!loader_) {
    return; //  no tiles loaded, don't do anything
  }
  
  //  get rid of old geometry, we will re-build this
  clearGeometry();

  if(scene_node_ != nullptr) {
      std::string parent_name = "null";
      if(scene_node_->getParentSceneNode() != nullptr) {
          parent_name = scene_node_->getParentSceneNode()->getName();
      }
      ROS_INFO_STREAM(__PRETTY_FUNCTION__<<" Rviz scene node: " << scene_node_->getName() << " with parent " << parent_name);

  } else {
      ROS_INFO_STREAM(__PRETTY_FUNCTION__<<" Rviz scene node: nullptr!");
  }

  //We can't assume that this lat,lon is @ (0,0) in the frame we have chosen!
  //GET TRANSFORM from sensor frame to fixed frame!
  ///@todo it's so anyoing that this transform lookup fails so often...
  tf::StampedTransform transform;
  try {
      tf::TransformListener * tf_listen = context_->getFrameManager()->getTFClient();
      tf_listen->lookupTransform(frame_property_->getFrameStd(),
                                 ref_fix_.header.frame_id,
                                 ref_fix_.header.stamp,
                                 transform);
      origin_x_in_frame_ = transform.getOrigin().x();
      origin_y_in_frame_ = transform.getOrigin().y();
      ROS_INFO_STREAM("Reference point corresponds to sensor (frame = " <<
                      ref_fix_.header.frame_id << ") at postition = " <<
                      origin_x_in_frame_ << ", " << origin_y_in_frame_ <<
                      " in " << frame_property_->getFrameStd());
  } catch(std::exception & e) {
      ROS_WARN_STREAM(e.what());
      ROS_WARN_STREAM("Setting origin_in_frame_ = 0,0!");
      origin_x_in_frame_ = 0.0;
      origin_y_in_frame_ = 0.0;
  }
  
  //  iterate over all tiles and create an object for each of them
  for (const TileLoader::MapTile &tile : loader_->tiles()) {
    // NOTE(gareth): We invert the y-axis so that positive y corresponds
    // to north. We are in XYZ->ENU convention here.
    const int w_pixels = tile.image().width();
    const int h_pixels = tile.image().height();
    const double tile_w_meters = w_pixels * loader_->resolution();
    const double tile_h_meters = h_pixels * loader_->resolution();

    // Shift back such that (0, 0) corresponds to the exact latitude and
    // longitude the tile loader requested.
    // This is the local origin, in the frame of the map node.

    //originOffset is the fraction of tile_x/y that the first lat/lon was at
    //i.e. the distance from top left corner

    double const dist_origin_from_left = loader_->originOffsetX() * tile_w_meters;
    double const dist_origin_from_top  = loader_->originOffsetY() * tile_h_meters;
    ROS_INFO_THROTTLE(1,"dist_origin_from_left = %f dist_origin_from_top = %f tile_width_m = %f tile_height_m = %f",
                      dist_origin_from_left,dist_origin_from_top,tile_w_meters,tile_h_meters);
    //double const dist_origin_from_bot  = (1.0 - loader_->originOffsetY()) * tile_h_meters;

    //const double origin_x = -dist_origin_from_left;
    //const double origin_y = -dist_origin_from_bot;

    // determine location of this tile, flipping y in the process
    double x_tile_tl = (tile.x() - loader_->centerTileX()) * tile_w_meters - dist_origin_from_left;
    //const double y = -(tile.y() - loader_->centerTileY()) * tile_h_meters - dist_origin_from_bot;


    double y_tile_tl = -(tile.y() - loader_->centerTileY()) * tile_h_meters + dist_origin_from_top;
    //all tiles are shifted one step up, can't really figure out why
    //y -= tile_h_meters;

    //account for position of sensor in map frame (only really works if that frame has Y=NORTH)
    x_tile_tl += origin_x_in_frame_;
    y_tile_tl += origin_y_in_frame_;

    //  don't re-use any ids
    const std::string name_suffix =
        std::to_string(tile.x()) + "_" + std::to_string(tile.y()) + "_" +
        std::to_string(map_id_) + "_" + std::to_string(scene_id_);

    if (tile.hasImage()) {
      //  one material per texture
      Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
          "material_" + name_suffix,
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      material->setReceiveShadows(false);
      material->getTechnique(0)->setLightingEnabled(false);
      material->setDepthBias(-16.0f, 0.0f);
      material->setCullingMode(Ogre::CULL_NONE);
      material->setDepthWriteEnabled(false);

      //  create textureing unit
      Ogre::Pass *pass = material->getTechnique(0)->getPass(0);
      Ogre::TextureUnitState *tex_unit = nullptr;
      if (pass->getNumTextureUnitStates() > 0) {
        tex_unit = pass->getTextureUnitState(0);
      } else {
        tex_unit = pass->createTextureUnitState();
      }
      // For debug purposes, draw borders and lines across the image
      QImage image_copy(tile.image());
      image_copy = image_copy.convertToFormat(QImage::Format_RGB888);
      QPainter painter(&image_copy);
      painter.setBrush(Qt::NoBrush);
      QPen pen;
      pen.setWidth(2);
      pen.setColor(Qt::lightGray);
      painter.setPen(pen);
      int h = image_copy.size().height();
      int w = image_copy.size().width();
      painter.drawRect(0,0,h,w);

      pen.setWidth(2);
      pen.setColor(Qt::lightGray);
      pen.setStyle(Qt::DashLine);
      painter.setPen(pen);
      painter.drawLine(0,h/2,w-1,h/2);
      painter.drawLine(w/2,0,w/2,h-1);

      if( tile.x() == loader_->centerTileX() && tile.y() == loader_->centerTileY() ) {
          pen.setColor(Qt::red);
          pen.setStyle(Qt::SolidLine);
          painter.setPen(pen);
          //draw start poistion on the image
          int start_pos_x_pixels = loader_->originOffsetX()*w_pixels;
          int start_pos_y_pixels = loader_->originOffsetY()*h_pixels;
          painter.drawLine(start_pos_x_pixels-4,start_pos_y_pixels+4,start_pos_x_pixels+4,start_pos_y_pixels-4);
          painter.drawLine(start_pos_x_pixels-4,start_pos_y_pixels-4,start_pos_x_pixels+4,start_pos_y_pixels+4);
      }

      painter.end();

      Ogre::TexturePtr texture =
          textureFromImage(image_copy, "texture_" + name_suffix);

      //  only add if we have a texture for it
      /*
      Ogre::TexturePtr texture =
          textureFromImage(tile.image(), "texture_" + name_suffix);
          */
      tex_unit->setTextureName(texture->getName());
      tex_unit->setTextureFiltering(Ogre::TFO_BILINEAR);

      //  create an object
      const std::string obj_name = "object_" + name_suffix;
      Ogre::ManualObject *obj = scene_manager_->createManualObject(obj_name);
      scene_node_->attachObject(obj);

      //  configure depth & alpha properties
      if (alpha_ >= 0.9998) {
        material->setDepthWriteEnabled(!draw_under_);
        material->setSceneBlending(Ogre::SBT_REPLACE);
      } else {
        material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        material->setDepthWriteEnabled(false);
      }

      if (draw_under_) {
        obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_3);
      } else {
        obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
      }

      tex_unit->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
                                  Ogre::LBS_CURRENT, alpha_);

      /* I tried to follow OGRE documentation but
       * Im doing something wrong...

      //  create a quad for this tile
      obj->begin(material->getName(), Ogre::RenderOperation::OT_TRIANGLE_STRIP);
      //Specify square as bl,br,tl,tr

      //bl
      obj->position(x_tile_tl,y_tile_tl-tile_h_meters,0.0f);
      obj->textureCoord(0.0f,1.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      //br
      obj->position(x_tile_tl+tile_w_meters,y_tile_tl-tile_h_meters,0.0f);
      obj->textureCoord(1.0f,1.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      //tl
      obj->position(x_tile_tl,y_tile_tl,0.0f);
      obj->textureCoord(0.0f,0.0f); //x,y
      obj->normal(0.0f, 0.0f, 1.0f);

      //tr
      obj->position(x_tile_tl+tile_w_meters,y_tile_tl,0.0f);
      obj->textureCoord(1.0f,0.0f);
      obj->normal(0.0f, 0.0f, 1.0f);


      obj->end();
      */

      //I don't really get the comments here...

      //  create a quad for this tile
      obj->begin(material->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

      //this might be the problem...
      //isn't (0,0) in slippy the top left?

      // bottom left???
      // top-left of texture???
      //obj->position(x_tile_tl, y_tile_tl, 0.0f);
      obj->position(x_tile_tl,y_tile_tl-tile_h_meters,0.0f);
      obj->textureCoord(0.0f, 0.0f); //0,0 of texture is top left?
                                     //1,1 of texture is bottom right?
      obj->normal(0.0f, 0.0f, 1.0f);

      // top right???
      // bottom right of texture
      //obj->position(x_tile_tl + tile_w_meters, y_tile_tl + tile_h_meters, 0.0f);
      obj->position(x_tile_tl + tile_w_meters, y_tile_tl, 0.0f);
      obj->textureCoord(1.0f, 1.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      // top left
      //obj->position(x_tile_tl, y_tile_tl + tile_h_meters, 0.0f);
      obj->position(x_tile_tl, y_tile_tl, 0.0f);
      obj->textureCoord(0.0f, 1.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      //  bottom left
      //obj->position(x_tile_tl, y_tile_tl, 0.0f);
      obj->position(x_tile_tl, y_tile_tl-tile_h_meters, 0.0f);
      obj->textureCoord(0.0f, 0.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      // bottom right
      //obj->position(x_tile_tl + tile_w_meters, y_tile_tl, 0.0f);
      obj->position(x_tile_tl + tile_w_meters, y_tile_tl-tile_h_meters, 0.0f);
      obj->textureCoord(1.0f, 0.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      // top right
      //obj->position(x_tile_tl + tile_w_meters, y_tile_tl + tile_h_meters, 0.0f);
      obj->position(x_tile_tl + tile_w_meters, y_tile_tl, 0.0f);
      obj->textureCoord(1.0f, 1.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      obj->end();

      if (draw_under_property_->getValue().toBool()) {
        //  render under everything else
        obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_3);
      }

      MapObject object;
      object.object = obj;
      object.texture = texture;
      object.material = material;
      objects_.push_back(object);
    }
  }
  scene_id_++;
}

void AerialMapDisplay::initiatedRequest(QNetworkRequest request) {
  ROS_INFO("Requesting %s", qPrintable(request.url().toString()));
}

void AerialMapDisplay::receivedImage(QNetworkRequest request) {
  ROS_INFO("Loaded tile %s", qPrintable(request.url().toString()));
}

void AerialMapDisplay::finishedLoading() {
  ROS_INFO("Finished loading all tiles.");
  dirty_ = true;
  setStatus(StatusProperty::Ok, "Message", "Loaded all tiles.");
  //  set property for resolution display
  if (loader_) {
    resolution_property_->setValue(loader_->resolution());
  }
}

void AerialMapDisplay::errorOcurred(QString description) {
  ROS_ERROR("Error: %s", qPrintable(description));
  setStatus(StatusProperty::Error, "Message", description);
}

// TODO(gareth): We are technically ignoring the orientation from the
// frame manager here - does this make sense?
//No IT DOES NOT!
void AerialMapDisplay::transformAerialMap() {
  // pass in identity to get pose of robot wrt to the fixed frame
  // the map will be shifted so as to compensate for the center tile shifting
  geometry_msgs::Pose pose;
  pose.orientation.w = 1;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;
  
  const std::string frame = frame_property_->getFrameStd();
  Ogre::Vector3 position{0, 0, 0};
  Ogre::Quaternion orientation{1, 0, 0, 0};

  //pose here is always (0,0,0)...

  //Transform a pose from a frame into the fixed_frame.
  // get the transform at the time we received the reference lat and lon

  //Okay, here we transform our "map" frame (if that is selected) to the fixed frame in rviz
  ROS_INFO_STREAM("Getting transform from "<<frame << " to fixed frame ("<<context_->getFixedFrame().toStdString()<<")");


  //this transform is way different from what TF tells us it is...
  //Let's not use ref_fix_.header.stamp but latest time... this helped...
  if (!context_->getFrameManager()->transform(frame, ros::Time(0), pose,
                                              position, orientation)) {
    ROS_WARN("Error transforming map '%s' from frame '%s' to frame '%s'",
             qPrintable(getName()), frame.c_str(), qPrintable(fixed_frame_));

    setStatus(StatusProperty::Error, "Transform",
              "No transform from [" + QString::fromStdString(frame) + "] to [" +
                  fixed_frame_ + "]");

    // set the transform to identity on failure
    position = Ogre::Vector3::ZERO;
    orientation = Ogre::Quaternion::IDENTITY;
  } else {
    setStatus(StatusProperty::Ok, "Transform", "Transform OK");
  }
  if (position.isNaN() || orientation.isNaN()) {
    // this can occur if an invalid TF is published. Set to identiy so OGRE does
    // not throw an assertion
    position = Ogre::Vector3::ZERO;
    orientation = Ogre::Quaternion::IDENTITY;
    ROS_ERROR("rviz_satellite received invalid transform, setting to identity");
  }

  //OKAY! Ogre uses a different coordinate sytem than rviz (yaw becomes roll)
  ROS_INFO_STREAM("GOT transform x,y,z = " << position.x << ", " <<position.y << "," <<position.z <<
                  " r,p,y (deg) = " << orientation.getRoll().valueDegrees()<<", "<<orientation.getPitch().valueDegrees() <<", " << orientation.getYaw().valueDegrees() );

  // Here we assume that the fixed/world frame is at altitude=0
  // force aerial imagery on ground
  position.z = 0;
  scene_node_->setPosition(position);
  
  const int convention = frame_convention_property_->getOptionInt();
  if (convention == FRAME_CONVENTION_XYZ_ENU) {
    // ENU corresponds to our default drawing method
    //scene_node_->setOrientation(Ogre::Quaternion::IDENTITY);
      scene_node_->setOrientation(orientation);
      //get yaw from it
      std::stringstream ss;
      ss << "Adjusted: xyz= " << position.x << ", " <<position.y << ", " << position.z << " r,p,y = "
         <<  orientation.getRoll().valueDegrees()<<", "<<orientation.getPitch().valueDegrees() <<", " << orientation.getYaw().valueDegrees();
      ROS_INFO_STREAM("Setting status msg to: "<<ss.str());
      setStatus(StatusProperty::Ok, "Transform", std::string(ss.str()).c_str());
  } else if (convention == FRAME_CONVENTION_XYZ_NED) {
    // NOTE(gareth): XYZ->NED will cause the map to appear reversed when viewed
    // from above (from +z).
    // clang-format off
    const Ogre::Matrix3 xyz_R_ned(0, 1, 0,
                                  1, 0, 0,
                                  0, 0,-1);
    // clang-format on
    scene_node_->setOrientation(xyz_R_ned.Transpose());
  } else if (convention == FRAME_CONVENTION_XYZ_NWU) {
    // clang-format off
    const Ogre::Matrix3 xyz_R_nwu(0,-1, 0,
                                  1, 0, 0,
                                  0, 0, 1);
    // clang-format on
    scene_node_->setOrientation(xyz_R_nwu.Transpose());
  } else {
    ROS_ERROR_STREAM("Invalid convention code: " << convention);
  }
}

void AerialMapDisplay::fixedFrameChanged() { transformAerialMap(); }

void AerialMapDisplay::reset() {
  Display::reset();
  //  unsub,clear,resub
  updateTopic();
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::AerialMapDisplay, rviz::Display)

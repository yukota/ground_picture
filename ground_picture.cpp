
#include <cstdint>
#include <cmath>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <sstream>

#define BOOST_NETWORK_ENABLE_HTTPS
#define BOOST_NETWORK_NO_LIB
#include <boost/network/protocol/http/client.hpp>
#include <boost/network/uri.hpp>
#include <boost/math/constants/constants.hpp>

#include <png++/png.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#include <gazebo/common/common.hh>

#include "ground_picture.hpp"

namespace gazebo
{
  GroundPicture::GroundPicture(){}

  GroundPicture::~GroundPicture(){}
  void GroundPicture::Load(rendering::VisualPtr visual, sdf::ElementPtr sdf)
  {

    ignition::math::Vector3d size = visual->GetGeometrySize();
    uint32_t width = size.X();
    uint32_t height = size.Y();

    // get map picture
    if (!sdf->HasElement("sdf")) {
      return;
    }
    sdf::ElementPtr sdf_element = sdf->GetElement("sdf");
    if(!sdf_element->HasElement("apikey")) {
      gzerr << "write apikey element.\n";
      return;
    }
    std::string api_key = sdf_element->GetElement("apikey")->Get<std::string>();
    if(api_key.empty()) {
      gzerr << "write apikey element.\n";
      return;
    }

    if(!sdf_element->HasElement("longitude")) {
      gzerr << "write longitude element.\n";
      return;
    }
    double longitude = sdf_element->GetElement("longitude")->Get<double>();

    if(!sdf_element->HasElement("latitude")) {
      gzerr << "write latitude element.\n";
      return;
    }
    double latitude = sdf_element->GetElement("latitude")->Get<double>();

    // get resolution settings
    uint32_t texture_width;
    uint32_t texture_height;
    if(sdf_element->HasElement("pixels_per_meter")) {
      uint16_t pixels_per_meter = sdf_element->GetElement("pixels_per_meter")->Get<uint16_t>();
      texture_width = width * pixels_per_meter;
      texture_height = height * pixels_per_meter;
    } else {
      texture_width = width * 4;
      texture_height = height * 4;
    }

    GoogleMapReader map_reader(api_key, longitude, latitude, width, height, texture_width, texture_height);
    auto image = map_reader.Request();


    geo_visual_.reset(
      new GeoVisual("ground_picture", visual, texture_width, texture_height));
    geo_visual_->render(image);

  }

  // GeoVisual class
  GeoVisual::GeoVisual(const std::string &name, rendering::VisualPtr parent, uint32_t height, uint32_t width)
    :rendering::Visual(name, parent), height_(height), width_(width) {
    texture_ = Ogre::TextureManager::getSingleton().createManual(
        name + "__GroundTexture__",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,
        width_,
        height_,
        0,
        Ogre::PF_BYTE_BGRA,
        Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

    Ogre::MaterialPtr material =
      Ogre::MaterialManager::getSingleton().create(
          name + "__GroundMaterial__",
          "General");

    material->getTechnique(0)->getPass(0)->createTextureUnitState(
        name + "__GroundTexture__");
    material->setReceiveShadows(false);

    double factor = 1.0;

    Ogre::ManualObject mo(name + "__GroundObject__");
    mo.begin(name + "__GroundMaterial__",
        Ogre::RenderOperation::OT_TRIANGLE_LIST);

    mo.position(-factor / 2, factor / 2, 0.51);
    mo.textureCoord(0, 0);

    mo.position(factor / 2, factor / 2, 0.51);
    mo.textureCoord(1, 0);

    mo.position(factor / 2, -factor / 2, 0.51);
    mo.textureCoord(1, 1);

    mo.position(-factor / 2, -factor / 2, 0.51);
    mo.textureCoord(0, 1);

    mo.triangle(0, 3, 2);
    mo.triangle(2, 1, 0);
    mo.end();

    mo.convertToMesh(name + "__GroundMesh__");

    Ogre::MovableObject *obj = (Ogre::MovableObject*)
      this->GetSceneNode()->getCreator()->createEntity(
          name + "__GroundEntity__",
          name + "__GroundMesh__");
    obj->setCastShadows(false);

    this->AttachObject(obj);
  }

  GeoVisual::~GeoVisual() {}

  void GeoVisual::render(const cv::Mat &src_image)
  {

    cv::Mat bgra_image;
    cv::cvtColor(src_image, bgra_image, CV_RGBA2BGRA);

    // Get pixel buffer
    Ogre::HardwarePixelBufferSharedPtr pixelBuffer =
      this->texture_->getBuffer();

    // lock rendering
    pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
    const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
    uint8_t* pDest = static_cast<uint8_t*>(pixelBox.data);
    memcpy(pDest, bgra_image.data, height_ * width_ * 4);
    // unlock rendering
    pixelBuffer->unlock();
  }



  GoogleMapReader::GoogleMapReader(const std::string &apikey,
      const double &latitude,
      const double &longitude,
      const int32_t &field_width,
      const int32_t &field_height,
      const uint32_t &texture_width,
      const uint32_t &texture_height) :
    api_key_(apikey), latitude_(latitude), longitude_(longitude), field_width_(field_width), field_height_(field_height), texture_width_(texture_width), texture_height_(texture_height)
  {
    SetupRequestParmeters();

  }

  cv::Mat GoogleMapReader::Request() {
    std::string url = CreateGoogleMapApiUrl();
    gzdbg << "Get ground picture from "url << "\n";

    boost::network::http::client::request request(url);
    request << boost::network::header("Connection", "close");

    boost::network::http::client client;
    boost::network::http::client::response response = client.get(request);
    std::stringstream body_stream;

    body_stream << boost::network::http::body(response);

    png::image<png::rgb_pixel>load_image(body_stream);

    // convert png::rgb_pixel to cvMat
    uint32_t load_image_width = load_image.get_width();
    uint32_t load_image_height = load_image.get_width();

    cv::Mat image = cv::Mat(load_image_height, load_image_width, CV_8UC3);
    // convert png to cvMat
    for(uint32_t pixel_width = 0; pixel_width < load_image_width; ++pixel_width) {
      for(uint32_t pixel_height = 0; pixel_height < load_image_height; ++pixel_height) {
        auto pixel = load_image.get_pixel(pixel_width, pixel_height);
        image.at<cv::Vec3b>(cv::Point(pixel_width, pixel_height)) = cv::Vec3b(pixel.blue, pixel.green, pixel.red);
      }
    }

    // resize to texture size
    cv::Mat resized_image = cv::Mat(texture_height_, texture_width_, CV_8UC3);
    cv::resize(image, resized_image, resized_image.size());
    return resized_image;
  }

  std::string GoogleMapReader::CreateGoogleMapApiUrl() const {
    std::stringstream ss;
    ss << "https://maps.googleapis.com/maps/api/staticmap?";
    ss << "maptype=satellite&";
    ss << "center=" << std::fixed << std::setprecision(6) << latitude_ << "," << std::fixed << std::setprecision(6) << longitude_ << "&";
    ss << "zoom=" << request_zoom_rate_ << "&";
    ss << "size="<< request_width_ << "x" << request_height_ << "&";
    ss << "key=" << api_key_;
    return ss.str();
  }

  uint16_t GoogleMapReader::MetersToEquatorPixels(const double &meters,
      const double &latitude,
      const uint16_t &zoom_rate) const {
    double rad = latitude_ * boost::math::constants::pi<double>() / 180.0;

    double metersPerPixel = 40075016.68 / (256.0 * std::pow(2, zoom_rate));
    double pixels = meters / std::cos(rad) / metersPerPixel;
    uint16_t upixels = static_cast<uint16_t>(pixels);
    return upixels;
  }


  void GoogleMapReader::SetupRequestParmeters() {
    // search prefferd zoom rate.
    // zooming level of google map is between 0 and 21.
    for(uint16_t zoom_rate = 0; zoom_rate <= 21; ++zoom_rate) {
      uint16_t pixel_width = MetersToEquatorPixels(field_width_, latitude_, zoom_rate);
      uint16_t pixel_height = MetersToEquatorPixels(field_height_, latitude_, zoom_rate);
      if ((pixel_width > texture_width_) && (pixel_height > texture_height_)) {
        // set request parameters as prefferd.
        request_zoom_rate_ = zoom_rate;
        request_width_ = pixel_width;
        request_height_ = pixel_height;
        return;
      }
    }

    request_zoom_rate_ = 21;
    request_width_ = MetersToEquatorPixels(field_width_, latitude_, 21);
    request_height_ = MetersToEquatorPixels(field_height_, latitude_, 21);
    return;
  }

  GZ_REGISTER_VISUAL_PLUGIN(GroundPicture)
}

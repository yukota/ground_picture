#ifndef GROUND_PICTURE_H
#define GROUND_PICTURE_H

#include <memory>
#include <string>
#include <cstdint>

#include <opencv2/core/core.hpp>

#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>

namespace gazebo
{

  class GoogleMapReader
  {
    public:
      GoogleMapReader(const std::string &apikey, const double &latitude, const double &longitude, const int32_t &field_width, const int32_t &field_height, const uint32_t &texture_width, const uint32_t &texture_height);

      cv::Mat Request();

    private:
      std::string api_key_;

      // center of map.
      double latitude_;
      double longitude_;

      int32_t field_height_;
      int32_t field_width_;

      uint32_t texture_height_;
      uint32_t texture_width_;

      uint16_t request_zoom_rate_;
      uint16_t request_width_;
      uint16_t request_height_;


      std::string CreateGoogleMapApiUrl() const;

      void SetupRequestParmeters();
      uint16_t MetersToEquatorPixels(const double &meters,
          const double &latitude,
          const uint16_t &zoom_rate) const;

  };

  class GeoVisual : public rendering::Visual
  {
    public:
     GeoVisual(const std::string &name,
         rendering::VisualPtr parent,
         uint32_t height, uint32_t width);

     virtual ~GeoVisual();

     void render(const cv::Mat &src_image);

    private:
     uint32_t height_;
     uint32_t width_;

     Ogre::TexturePtr texture_;

  };

  class GroundPicture : public VisualPlugin
  {
    public:
      GroundPicture();
      virtual ~GroundPicture();

      void Load(rendering::VisualPtr visual, sdf::ElementPtr sdf);

    private:
      std::shared_ptr<GeoVisual> geo_visual_;
  };
}

#endif

// rviz_ogre_vendor exposes OGRE headers directly on the include path.
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <tf2_ros/transform_listener.h>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include "apfrl_rviz_plugins/obstacles_visual.hpp"
#include "apfrl_rviz_plugins/obstacles_display.hpp"
#include <memory>
#include <rclcpp/logging.hpp>
namespace apfrl_rviz_plugins
{
    bool is_entered_first = true;

    ObstaclesDisplay::ObstaclesDisplay()
    {
        m_colour_property = new rviz_common::properties::ColorProperty("Color", QColor(255, 0, 0),
                                                                       "Color to draw the acceleration arrows.",
                                                                       this, SLOT(updateColorAndAlpha()));

        m_alpha_property = new rviz_common::properties::FloatProperty("Alpha", 1.0,
                                                                      "0 is fully transparent, 1.0 is fully opaque.",
                                                                      this, SLOT(updateColorAndAlpha()));

        m_show_text_property = new rviz_common::properties::BoolProperty("Object text", true,
                                                                         "show some text about the object",
                                                                         this);
    }

    void ObstaclesDisplay::onInitialize()
    {
        MFDClass::onInitialize();
    }

    ObstaclesDisplay::~ObstaclesDisplay()
    {
    }

    void ObstaclesDisplay::reset()
    {
        MFDClass::reset();
        m_visual = nullptr;
    }

    void ObstaclesDisplay::updateColorAndAlpha()
    {
        float alpha = m_alpha_property->getFloat();
        Ogre::ColourValue color = m_colour_property->getOgreColor();
        if (m_visual == nullptr)
            return;
        m_visual->setColor(color.r, color.g, color.b, alpha);
    }

    void ObstaclesDisplay::processMessage(const apfrl_interfaces::msg::Obstacles::ConstSharedPtr msg)
    {
        Ogre::Quaternion orientation;
        Ogre::Vector3 position;
        if (!context_->getFrameManager()->getTransform(msg->header.frame_id,
                                                       msg->header.stamp,
                                                       position, orientation))
        {

            setStatus(rviz_common::properties::StatusProperty::Error, "Transforms",
                      QString("Error transforming from ") + msg->header.frame_id.c_str() + " to " + qPrintable(fixed_frame_));
            return;
        }

        auto visual = std::make_shared<ObstaclesVisual>(context_->getSceneManager(), scene_node_);
        visual->setShowText(m_show_text_property->getBool());
        visual->setMessage(msg);

        visual->setFramePosition(position);
        visual->setFrameOrientation(orientation);

        float alpha = m_alpha_property->getFloat();
        Ogre::ColourValue color = m_colour_property->getOgreColor();
        visual->setColor(color.r, color.g, color.b, alpha); // blues

        std::swap(visual, m_visual);
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(apfrl_rviz_plugins::ObstaclesDisplay, rviz_common::Display)

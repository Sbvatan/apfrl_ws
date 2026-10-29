#include "apfrl_rviz_plugins/obstacles_visual.hpp"
namespace apfrl_rviz_plugins
{

    ObstaclesVisual::ObstaclesVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node)
    {
        m_scene_manager = scene_manager;
        m_parent_node = parent_node;
        m_frame_node = parent_node->createChildSceneNode();
        m_shapes.reserve(400);
    }

    ObstaclesVisual::~ObstaclesVisual()
    {
        m_scene_manager->destroySceneNode(m_frame_node);
    }

    void ObstaclesVisual::setMessage(const apfrl_interfaces::msg::Obstacles::ConstSharedPtr &msg)
    {
        m_shapes.clear();

        for (auto &text_node : m_text_nodes)
          m_scene_manager->destroySceneNode(text_node);

        m_text_nodes.clear();
        m_text_objects.clear();

        for (auto &object : msg->obs)
        {
            auto shape = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Cylinder, m_scene_manager, m_frame_node);
            Ogre::Vector3 position;
            if (std::isnan(object.pose.x) || std::isnan(object.pose.y))
            {
                continue;
            }
            position.x = object.pose.x;
            position.y = object.pose.y;
            position.z = 0.5;
            shape->setPosition(position);


            Ogre::Vector3 dir(Ogre::Real(1.0), Ogre::Real(0.0), Ogre::Real(0.0));
            Ogre::Radian angle(Ogre::Real(M_PI_2));
            Ogre::Vector3 dir_yaw(Ogre::Real(0.0), Ogre::Real(1.0), Ogre::Real(0.0));
            Ogre::Radian angle_yaw(Ogre::Real(object.pose.theta));
            Ogre::Quaternion q(angle, dir);
            Ogre::Quaternion q_yaw(angle_yaw, dir_yaw);
            shape->setOrientation(q * q_yaw);

            Ogre::Vector3 scale;
            scale.x = 2 * object.r_a;
            scale.y = 1.0;
            scale.z = 2 * object.r_b;

            shape->setScale(scale);
            m_shapes.push_back(shape);
        }
    }

    void ObstaclesVisual::setFramePosition(const Ogre::Vector3 &position)
    {
        m_frame_node->setPosition(position);
    }

    void ObstaclesVisual::setFrameOrientation(const Ogre::Quaternion &orientation)
    {
        m_frame_node->setOrientation(orientation);
    }

    void ObstaclesVisual::setColor(float r, float g, float b, float a)
    {

      ObstaclesVisual::color_r = r;
      ObstaclesVisual::color_g = g;
      ObstaclesVisual::color_b = b;
      ObstaclesVisual::color_alpha = a;

      for (size_t i = 0; i < m_shapes.size(); i++)
      {

        if (i == 0)
            m_shapes[i]->setColor(r, g, b, 0.1f*a);
        else
            m_shapes[i]->setColor(r, g, b, a);
      }
    }
}

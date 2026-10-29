#pragma once
// rviz_ogre_vendor puts OGRE headers directly on the include path.
#include <OgreVector3.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include "apfrl_interfaces/msg/obstacle.hpp"
#include "apfrl_interfaces/msg/obstacles.hpp"
#include <tf2/transform_datatypes.h>
#include "rviz_rendering/objects/shape.hpp"

#include <algorithm>
#include <cmath>

namespace rviz_common
{
    class Shape;

    class MovableText;

}

namespace apfrl_rviz_plugins
{

    class ObstaclesVisual
    {
    public:
        float color_r;
        float color_g;
        float color_b;
        float color_alpha;

        // Acceptable values of colors, due to the bug
        // in the rviz-plugin , they sometimes have extremely big numbers
        float color_r_legit;
        float color_g_legit;
        float color_b_legit;
        float color_alpha_legit;
        bool is_entered_first = true;

        ObstaclesVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node);

        virtual ~ObstaclesVisual();

        void setMessage(const apfrl_interfaces::msg::Obstacles::ConstSharedPtr &msg);

        void setFramePosition(const Ogre::Vector3 &position);

        void setFrameOrientation(const Ogre::Quaternion &orientation);

        void setColor(float r, float g, float b, float a);

        void setShowText(bool b)
        {
            m_show_text = b;
        }

    private:
        std::vector<std::shared_ptr<rviz_rendering::Shape>> m_shapes;
        bool m_show_text;

        Ogre::SceneNode *m_frame_node;

        std::vector<Ogre::SceneNode *> m_text_nodes;
        std::vector<std::shared_ptr<rviz_common::MovableText>> m_text_objects;

        Ogre::SceneManager *m_scene_manager;
        Ogre::SceneNode *m_parent_node;
    };

}

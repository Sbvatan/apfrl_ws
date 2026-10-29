#pragma once

#ifndef Q_MOC_RUN

#include <rviz_common/message_filter_display.hpp>
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include <apfrl_interfaces/msg/obstacle.hpp>
#include <apfrl_interfaces/msg/obstacles.hpp>
#endif

namespace Ogre
{
    class SceneNode;
}

namespace rviz_common
{
    class ColorProperty;

    class FloatProperty;

    class IntProperty;
}

namespace apfrl_rviz_plugins
{

    class ObstaclesVisual;

    class ObstaclesDisplay : public rviz_common::MessageFilterDisplay<apfrl_interfaces::msg::Obstacles>
    {
        Q_OBJECT
    public:
        ObstaclesDisplay();

        virtual ~ObstaclesDisplay();

    protected:
        virtual void onInitialize() override;
        virtual void reset() override;

    private Q_SLOTS:
        void updateColorAndAlpha();

    private:
        void processMessage(const apfrl_interfaces::msg::Obstacles::ConstSharedPtr msg) override;

        // Storage for the visual.
        std::shared_ptr<ObstaclesVisual> m_visual;

        rviz_common::properties::ColorProperty *m_colour_property;
        rviz_common::properties::FloatProperty *m_alpha_property;
        rviz_common::properties::BoolProperty *m_show_text_property;
    };

}

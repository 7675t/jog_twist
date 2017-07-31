#include "jog_twist_panel.h"
#include <boost/thread.hpp>
#include <rviz/config.h>
#include <ros/package.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSignalMapper>

namespace jog_twist
{

  JogTwistPanel::JogTwistPanel(QWidget* parent)
    : rviz::Panel(parent)
  {
    QHBoxLayout* frame_layout = new QHBoxLayout;
    frame_layout->addWidget( new QLabel( "Frame:" ));
    frame_cbox_ = new QComboBox();
    frame_layout->addWidget(frame_cbox_);

    QHBoxLayout* axis_layout = new QHBoxLayout;
    axis_layout->addWidget( new QLabel( "Axis:" ));
    axis_cbox_ = new QComboBox();
    axis_layout->addWidget(axis_cbox_);

    QHBoxLayout* jog_layout = new QHBoxLayout;
    jog_layout->addWidget( new QLabel( "Jog:" ));
    jog_slider_ = new QSlider(Qt::Horizontal);
    jog_layout->addWidget(jog_slider_);

    QHBoxLayout* pos_x_layout = new QHBoxLayout;
    pos_x_layout->addWidget( new QLabel( "X:" ));
    pos_x_text_ = new QLCDNumber();
    pos_x_layout->addWidget(pos_x_text_);

    QHBoxLayout* pos_y_layout = new QHBoxLayout;
    pos_y_layout->addWidget( new QLabel( "Y:" ));
    pos_y_text_ = new QLCDNumber();
    pos_y_layout->addWidget(pos_y_text_);

    QHBoxLayout* pos_z_layout = new QHBoxLayout;
    pos_z_layout->addWidget( new QLabel( "Z:" ));
    pos_z_text_ = new QLCDNumber();
    pos_z_layout->addWidget(pos_z_text_);

    QVBoxLayout* layout = new QVBoxLayout;
    layout->addLayout(frame_layout);
    layout->addLayout(axis_layout);
    layout->addLayout(jog_layout);
    layout->addLayout(pos_x_layout);
    layout->addLayout(pos_y_layout);
    layout->addLayout(pos_z_layout);
    setLayout(layout);

    connect(frame_cbox_, SIGNAL(currentIndexChanged()), this, SLOT(respondFrameSelect()));
    connect(axis_cbox_, SIGNAL(currentIndexChanged()), this, SLOT(respondAxisSelect));
  }

  void JogTwistPanel::onInitialize()
  {
    ros::NodeHandle nh;
  }

  void JogTwistPanel::respondFrameSelect()
  {
    boost::mutex::scoped_lock lock(mutex_);
    frame_id_ = "frame_id";
  }

  void JogTwistPanel::respondAxisSelect()
  {
    boost::mutex::scoped_lock lock(mutex_);
    axis_id_ = "axis_id";
  }

  void JogTwistPanel::save(rviz::Config config) const
  {
    rviz::Panel::save(config);
  }

  void JogTwistPanel::load(const rviz::Config& config)
  {
    rviz::Panel::load(config);
  }

}  // namespace jog_twist


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jog_twist::JogTwistPanel, rviz::Panel)

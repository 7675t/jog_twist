#include "jog_twist_panel.h"
#include <boost/thread.hpp>
#include <rviz/config.h>
#include <rviz/visualization_manager.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSignalMapper>
#include <QTimer>

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

  connect(frame_cbox_, SIGNAL(activated()), this, SLOT(respondFrame));
  connect(axis_cbox_, SIGNAL(activated()), this, SLOT(respondAxis));

  // Timer for update Frame ComboBox
  QTimer* output_timer = new QTimer( this );
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( updateFrame() ));
  output_timer->start(10000);  
}

void JogTwistPanel::onInitialize()
{
  updateFrame();
  initAxisComboBox();
}

void JogTwistPanel::updateFrame()
{
  typedef std::vector<std::string> V_string;
  V_string frames;
  vis_manager_->getTFClient()->getFrameStrings( frames );
  std::sort(frames.begin(), frames.end());
  frame_cbox_->clear();
  for (V_string::iterator it = frames.begin(); it != frames.end(); ++it )
  {
    const std::string& frame = *it;
    if (frame.empty())
    {
      continue;
    }
    frame_cbox_->addItem(it->c_str());
  }
}  

void JogTwistPanel::respondFrame()
{
  boost::mutex::scoped_lock lock(mutex_);
  frame_id_ = frame_cbox_->currentText().toStdString();
}

void JogTwistPanel::respondAxis()
{
  boost::mutex::scoped_lock lock(mutex_);
  axis_id_ = axis_cbox_->currentText().toStdString();
}

void JogTwistPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void JogTwistPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}

void JogTwistPanel::initAxisComboBox()
{
  axis_cbox_->addItem("x");
  axis_cbox_->addItem("y");
  axis_cbox_->addItem("z");
}  

}  // namespace jog_twist


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jog_twist::JogTwistPanel, rviz::Panel)

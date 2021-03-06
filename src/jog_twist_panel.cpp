#include "jog_twist_panel.h"
#include <boost/thread.hpp>
#include <rviz/config.h>
#include <rviz/visualization_manager.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TwistStamped.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSignalMapper>
#include <QTimer>

namespace jog_twist
{

JogTwistPanel::JogTwistPanel(QWidget* parent)
  : rviz::Panel(parent), jog_value_(0)
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
  jog_slider_->setTickPosition(QSlider::TicksBelow);
  jog_slider_->setTickInterval(500);
  jog_slider_->setMinimum(-1000);
  jog_slider_->setMaximum( 1000);
  jog_slider_->setTracking(true);
  // How can I change the slider style more properly??
  jog_slider_->setStyleSheet("QSlider::handle {"
                             "background: white;"
                             "border: 3px solid black;"
                             "width: 60px;"
                             "margin: -30px 0;"
                             "} "
                             "QSlider::sub-page {"
                             "background: rgb(164, 192, 2);"
                             "} "
                             "QSlider::add-page {"
                             "background: rgb(223, 70, 70);"
                             "} ");
  jog_layout->addWidget(jog_slider_);

  QHBoxLayout* pos_x_layout = new QHBoxLayout;
  pos_x_layout->addWidget( new QLabel( "X:" ));
  pos_x_text_ = makeNumericLabel();
  pos_x_layout->addWidget(pos_x_text_);

  QHBoxLayout* pos_y_layout = new QHBoxLayout;
  pos_y_layout->addWidget( new QLabel( "Y:" ));
  pos_y_text_ = makeNumericLabel();
  pos_y_layout->addWidget(pos_y_text_);

  QHBoxLayout* pos_z_layout = new QHBoxLayout;
  pos_z_layout->addWidget( new QLabel( "Z:" ));
  pos_z_text_ = makeNumericLabel();
  pos_z_layout->addWidget(pos_z_text_);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(frame_layout);
  layout->addLayout(axis_layout);
  layout->addLayout(jog_layout);
  layout->addLayout(pos_x_layout);
  layout->addLayout(pos_y_layout);
  layout->addLayout(pos_z_layout);
  setLayout(layout);

  connect(frame_cbox_, SIGNAL(activated(int)), this, SLOT(respondFrame(int)));
  connect(axis_cbox_, SIGNAL(activated(int)), this, SLOT(respondAxis(int)));

  // Slider
  connect(jog_slider_, SIGNAL(valueChanged(int)), this, SLOT(respondSliderChanged(int)));
  connect(jog_slider_, SIGNAL(sliderReleased()), this, SLOT(respondSliderReleased()));

  // Timer for update Frame ComboBox
  QTimer* output_timer = new QTimer( this );
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( publish() ));
  output_timer->start(100);

  ros::NodeHandle nh;
  twist_pub_ = nh.advertise<geometry_msgs::TwistStamped>( "/cmd_vel", 1 );
  //pose_sub_ = nh_.subscribe(topic_name, 1, &PieChartDisplay::processMessage, this);
}

void JogTwistPanel::onInitialize()
{
  connect( vis_manager_, SIGNAL( preUpdate() ), this, SLOT( update() ));  
  updateFrame();
  initAxisComboBox();
}

void JogTwistPanel::update()
{
  tf::TransformListener* tf = vis_manager_->getTFClient();
  tf::StampedTransform transform;
  try{
    tf->lookupTransform(frame_id_, "link_j6", ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }  
  fillNumericLabel(pos_x_text_, transform.getOrigin().x());
  fillNumericLabel(pos_y_text_, transform.getOrigin().y());
  fillNumericLabel(pos_z_text_, transform.getOrigin().z());
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

void JogTwistPanel::publish()
{
  // publish
  geometry_msgs::TwistStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id_;
  if (axis_id_ == "x")
  {
    msg.twist.linear.x = jog_value_;
  }
  if (axis_id_ == "y")
  {
    msg.twist.linear.y = jog_value_;
  }
  if (axis_id_ == "z")
  {
    msg.twist.linear.z = jog_value_;
  }
  twist_pub_.publish(msg);
}  

void JogTwistPanel::respondFrame(int index)
{
  boost::mutex::scoped_lock lock(mutex_);
  frame_id_ = frame_cbox_->currentText().toStdString();
  ROS_INFO_STREAM("respondFrame: " << frame_id_);
}

void JogTwistPanel::respondAxis(int index)
{
  boost::mutex::scoped_lock lock(mutex_);
  axis_id_ = axis_cbox_->currentText().toStdString();
  ROS_INFO_STREAM("respondAxis: " << axis_id_);
}

void JogTwistPanel::respondSliderChanged(int value)
{
  boost::mutex::scoped_lock lock(mutex_);
  jog_value_ = value / 1000.0;
}

void JogTwistPanel::respondSliderReleased()
{
  jog_slider_->setValue(0);
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

QLineEdit* JogTwistPanel::makeNumericLabel()
{
  QLineEdit* label = new QLineEdit;
  label->setReadOnly( true );
  return label;
}

void JogTwistPanel::fillNumericLabel( QLineEdit* label, double value )
{
  label->setText( QString::number( value, 'f', 2 ));
}

}  // namespace jog_twist


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jog_twist::JogTwistPanel, rviz::Panel)

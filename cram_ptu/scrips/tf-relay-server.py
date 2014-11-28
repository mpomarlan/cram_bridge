#!/usr/bin/env python
#apparently not needed after catkin:
#import roslib
#roslib.load_manifest('cram_ptu')

import rospy

import actionlib
import tf

import std_msgs.msg
import cogman_msgs.msg

class TFRelayAction(object):
  # create messages that are used to publish result

  # NOTE: cogman_msgs package actually does not define any msg files; there is no TFRelayResult.msg in either ias_common or iai_common
  _result   = cogman_msgs.msg.TFRelayResult()

  def __init__(self, name):
    self._action_name = name
  # NOTE: cogman_msgs package actually does not define TFRelayAction
    self._as = actionlib.SimpleActionServer(self._action_name, cogman_msgs.msg.TFRelayAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    self._broadcaster = tf.TransformBroadcaster()
    
  def execute_cb(self, goal):
    # publish info to the console for the user
    rospy.loginfo('%s: Executing, publishing further transform to TF' % self._action_name)
    
    # let's broadcast
    trans = goal.transform.transform.translation
    rot = goal.transform.transform.rotation
    self._broadcaster.sendTransform((trans.x, trans.y, trans.z),
                                    (rot.x, rot.y, rot.z, rot.w),
                                    rospy.Time.now(),
                                    goal.transform.child_frame_id,
                                    goal.transform.header.frame_id)
      
    self._as.set_succeeded(self._result)
      
if __name__ == '__main__':
  rospy.init_node('tf_relay')
  TFRelayAction(rospy.get_name())
  rospy.spin()

;;; Copyright (c) 2013, Jan Winkler <winkler@cs.uni-bremen.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Universitaet Bremen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :cram-moveit)

(defun transform-stamped->msg (transform-stamped)
  (with-fields (stamp frame-id child-frame-id rotation translation) transform-stamped
    (make-message
     "geometry_msgs/TransformStamped"
     (stamp header) stamp
     (frame_id header) child-frame-id;frame-id
     (child_frame_id) frame-id;child-frame-id
     (x translation transform) (tf:x translation)
     (y translation transform) (tf:y translation)
     (z translation transform) (tf:z translation)
     (x rotation transform) (tf:x rotation)
     (y rotation transform) (tf:y rotation)
     (z rotation transform) (tf:z rotation)
     (w rotation transform) (tf:w rotation))))

(defun transform->msg (transform)
  (with-fields (rotation translation) transform
    (make-message
     "geometry_msgs/Transform"
     (x translation) (tf:x translation)
     (y translation) (tf:y translation)
     (z translation) (tf:z translation)
     (x rotation) (tf:x rotation)
     (y rotation) (tf:y rotation)
     (z rotation) (tf:z rotation)
     (w rotation) (tf:w rotation))))

(defun pose-distance (link-frame pose-stamped)
  "Returns the distance of stamped pose `pose-stamped' from the origin
coordinates of link `link-frame'. This can be for example used for
checking how far away a given grasp pose is from the gripper frame."
  (tf:v-dist (tf:make-identity-vector)
             (tf:origin (cl-tf2:ensure-pose-stamped-transformed
                         *tf2* pose-stamped link-frame :use-current-ros-time t))))

(defun motion-length (link-name planning-group pose-stamped
                        &key allowed-collision-objects
                          highlight-links)
  (let* ((pose-stamped-transformed
           (cl-tf2:ensure-pose-stamped-transformed
            *tf2* pose-stamped "/torso_lift_link" :use-current-ros-time t))
         (state-0 (moveit:plan-link-movement
                   link-name planning-group
                   pose-stamped-transformed
                   :allowed-collision-objects
                   allowed-collision-objects
                   :destination-validity-only t
                   :highlight-links highlight-links)))
    (when state-0
      (pose-distance
       link-name
       pose-stamped-transformed))))

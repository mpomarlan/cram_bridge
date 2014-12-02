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

(defvar *tf2* nil)
(defvar *tf-mutex* (sb-thread:make-mutex))

(defun unslash-frame (frame)
  (cond ((string= (elt frame 0) "/")
         (subseq frame 1))
        (t frame)))

(defun ensure-pose-stamped-transformable (pose-stamped target-frame
                                          &key ros-time)
  (sb-thread:with-mutex (*tf-mutex*)
    (let ((target-frame (unslash-frame target-frame))
          (source-frame (unslash-frame (tf:frame-id pose-stamped))))
      ;;(ros-info (moveit) "TF2 transform (~a -> ~a)"
      ;;          source-frame target-frame)
      (let ((first-run t))
        (loop for sleepiness = (or first-run (sleep 0.5))
              for time = (cond (ros-time (ros-time))
                               (t (tf:stamp pose-stamped)))
              for tst = (or first-run (ros-warn (moveit) "Retrying."))
              for can-tr = (cl-tf2:can-transform
                            *tf2*
                            target-frame
                            source-frame
                            time 2.0)
              when (progn
                     (when first-run
                       (setf first-run nil)
                       (setf ros-time t))
                     can-tr)
                do (return (tf:copy-pose-stamped pose-stamped :stamp time)))))))

(defun ensure-transform-available (reference-frame target-frame)
  (sb-thread:with-mutex (*tf-mutex*)
    (let ((target-frame (unslash-frame target-frame))
          (reference-frame (unslash-frame reference-frame)))
      (cpl:with-failure-handling
          ((cl-tf:tf-cache-error (f)
             (declare (ignore f))
             (ros-warn (moveit) "Failed to make transform available. Retrying.")
             (cpl:retry)))
        (let ((first-run t))
          (loop for sleepiness = (or first-run (sleep 1.0))
                for time = (ros-time)
                for can-tr = (let ((can-tr (cl-tf2:can-transform
                                            *tf2*
                                            target-frame
                                            reference-frame
                                            time 2.0)))
                               (when can-tr
                                 (tf:transform->stamped-transform
                                  reference-frame
                                  target-frame
                                  time
                                  (cl-tf2::transform can-tr))))
                when (progn
                       (setf first-run nil)
                       can-tr)
                  do (return can-tr)))))))

(defun ensure-pose-stamped-transformed (pose-stamped target-frame
                                        &key ros-time)
  (sb-thread:with-mutex (*tf-mutex*)
    (let ((target-frame (unslash-frame target-frame))
          (source-frame (unslash-frame (tf:frame-id pose-stamped))))
      ;; (ros-info (moveit) "TF2 transform (~a -> ~a)"
      ;;           source-frame target-frame)
      (cpl:with-failure-handling
          ((cl-tf:tf-cache-error (f)
             (declare (ignore f))
             (ros-warn (moveit) "Failed to transform pose (~a -> ~a). Retrying."
                       source-frame target-frame)
             (cpl:retry)))
        (let* ((rostime (cond (ros-time (roslisp:ros-time))
                              (t (tf:stamp pose-stamped))))
               (transform (cl-tf2:can-transform
                           *tf2*
                           target-frame
                           source-frame
                           rostime 3.0)))
          (unless transform
            (cpl:fail 'cl-tf:tf-cache-error))
          (let* ((cl-transforms-transform
                   (cl-tf2::transform transform))
                 (transformed-pose
                   (cl-transforms:transform-pose
                    cl-transforms-transform
                    pose-stamped)))
            (tf:make-pose-stamped
             target-frame
             rostime
             (cl-transforms:origin transformed-pose)
             (cl-transforms:orientation transformed-pose))))))))

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
             (tf:origin (ensure-pose-stamped-transformed
                         pose-stamped link-frame :ros-time t))))

(defun motion-length (link-name planning-group pose-stamped
                        &key allowed-collision-objects
                          highlight-links)
  (let* ((pose-stamped-transformed
           (ensure-pose-stamped-transformed
            pose-stamped "/torso_lift_link" :ros-time t))
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

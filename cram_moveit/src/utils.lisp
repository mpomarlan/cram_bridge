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

(defun ensure-pose-stamped-transformable (pose-stamped target-frame
                                          &key ros-time)
  (ros-info (moveit) "TF transform (~a -> ~a)"
            (tf:frame-id pose-stamped) target-frame)
  (let ((first-run t))
    (loop for sleepiness = (or first-run (sleep 0.5))
          for time = (cond (ros-time (ros-time))
                           (t (tf:stamp pose-stamped)))
          for tst = (or first-run (format t "Retry~%"))
          when (tf:wait-for-transform
                *tf*
                :source-frame (tf:frame-id pose-stamped)
                :target-frame target-frame
                :timeout 2.0
                :time time)
            do (setf first-run nil)
               (return (tf:copy-pose-stamped pose-stamped :stamp time)))))

(defun ensure-transform-available (reference-frame target-frame)
  (cpl:with-failure-handling
      ((cl-tf:tf-cache-error (f)
         (declare (ignore f))
         (ros-warn (moveit) "Failed to make transform available. Retrying.")
         (cpl:retry)))
    (let ((first-run t))
      (loop for sleepiness = (or first-run (sleep 1.0))
            for time = (ros-time)
            for transform = (when (tf:wait-for-transform
                                   *tf* :timeout 2.0
                                        :time time
                                        :source-frame reference-frame
                                        :target-frame target-frame)
                              (tf:lookup-transform
                               *tf* :time time
                                    :source-frame reference-frame
                                    :target-frame target-frame))
            when transform
              do (setf first-run nil)
                 (return transform)))))
  
(defun ensure-pose-stamped-transformed (pose-stamped target-frame
                                        &key ros-time)
  (cpl:with-failure-handling
      ((cl-tf:tf-cache-error (f)
         (declare (ignore f))
         (ros-warn (moveit) "Failed to transform pose. Retrying.")
         (cpl:retry)))
    (let ((rostime (cond (ros-time (roslisp:ros-time))
                         (t (tf:stamp pose-stamped)))))
      (unless (tf:wait-for-transform
               *tf*
               :timeout 1.5
               :time rostime
               :source-frame (tf:frame-id pose-stamped)
               :target-frame target-frame)
        (cpl:fail 'cl-tf:tf-cache-error))
      (tf:transform-pose
       *tf* :pose (ensure-pose-stamped-transformable
                   pose-stamped target-frame)
            :target-frame target-frame))))

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

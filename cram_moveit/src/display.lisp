;;; Copyright (c) 2014, Jan Winkler <winkler@cs.uni-bremen.de>
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
;;;     * Neither the name of the Universitaet Bremen nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(defparameter *robot-state-display-publisher* nil)
(defvar *robot-state-display-topic* "/display_robot_state")
(defparameter *object-colors* nil)
(defparameter *planning-scene-publisher* nil
  "Publisher handle for the planning scene topic.")

(defun display-robot-state (state &key highlight)
  (when (and *robot-state-display-publisher* state)
    (let ((highlights
            (map 'vector
                 (lambda (link-name)
                   (roslisp:make-message
                    "moveit_msgs/ObjectColor"
                    :id link-name
                    :color (roslisp:make-message
                            "std_msgs/ColorRGBA"
                            :r 1.0
                            :g 1.0
                            :b 0.0
                            :a 1.0)))
                 highlight)))
      (roslisp:publish
       *robot-state-display-publisher*
       (roslisp:make-message "moveit_msgs/DisplayRobotState"
                             :state state
                             :highlight_links highlights)))))

(defun publish-object-colors ()
  (let* ((msgs
           (loop for assignment in *object-colors*
                 collecting (destructuring-bind
                                (object-name . (r g b a)) assignment
                              (make-message
                               "moveit_msgs/ObjectColor"
                               (id) (string-upcase (string object-name))
                               (r color) r
                               (g color) g
                               (b color) b
                               (a color) a))))
         (msg (make-message
               "moveit_msgs/PlanningScene"
               (object_colors) (map 'vector #'identity msgs)
               (is_diff) t)))
    (roslisp:publish *planning-scene-publisher* msg)))

(defun clear-object-colors ()
  (prog1
      (setf *object-colors* nil)
    (publish-object-colors)))

(defun set-object-color (name color)
  (set-object-colors `(,(cons name color))))

(defun set-object-colors (assignments)
  (prog1
      (setf
       *object-colors*
       (append (remove-if (lambda (assignment)
                            (find assignment assignments
                                  :test (lambda (x y)
                                          (eql (car x) (car y)))))
                          *object-colors*)
               assignments))
    (publish-object-colors)))

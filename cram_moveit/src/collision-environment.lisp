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

(defclass collision-object ()
  ((name :initform nil
         :initarg :name
         :reader :name)
   (pose :initform nil
         :initarg :pose
         :reader :pose)
   (color :initform nil
          :initarg :color
          :reader :color)
   (primitive-shapes :initform nil
                     :initarg :primitive-shapes
                     :reader :primitive-shapes)
   (mesh-shapes :initform nil
                :initarg :mesh-shapes
                :reader :mesh-shapes)
   (plane-shapes :initform nil
                 :initarg :plane-shapes
                 :reader :plane-shapes)))

(defvar *known-collision-objects* nil
  "List of collision object instances registered with the CRAM/MoveIt!
bridge.")

(defgeneric register-collision-object (object &rest rest))

(defmethod register-collision-object ((object object-designator)
                                      &key pose-stamped add)
  (flet ((primitive-code (symbol)
           (let ((symbol (or symbol :box)))
             (roslisp-msg-protocol:symbol-code
              'shape_msgs-msg:solidprimitive symbol))))
    (let* ((name (string-upcase (string (desig-prop-value
                                         object 'desig-props:name))))
           (shape-prop (or (desig-prop-value object 'desig-props:shape)
                           'desig-props:box))
           (shape (primitive-code
                   (case shape-prop
                     (desig-props:box :box)
                     (desig-props:sphere :sphere)
                     (desig-props:cylinder :cylinder)
                     (desig-props:round :cylinder)
                     (desig-props:cone :cone))))
           (dimensions (or
                        (desig-prop-value object 'desig-props:dimensions)
                        (case shape-prop
                          (desig-props:box (vector 0.1 0.1 0.1))
                          (desig-props:sphere (vector 0.1 0.1))
                          (desig-props:cylinder (vector 0.2 0.03))
                          (desig-props:round (vector 0.2 0.08))
                          (desig-props:cone (vector 0.1 0.1)))))
           (pose-stamped
             (or pose-stamped
                 (when (desig-prop-value object 'desig-props:at)
                   (reference (desig-prop-value object 'desig-props:at))))))
      (unless pose-stamped
        (roslisp:ros-warn (moveit) "No pose-stamped given (neither manually nor in the object-designator) when adding object-designator ~a to the collision environment." object))
      (register-collision-object
       name
       :primitive-shapes (list (roslisp:make-msg
                                "shape_msgs/SolidPrimitive"
                                type shape
                                dimensions
                                (cond ((eql shape-prop 'desig-props:round)
                                       (vector (elt dimensions 2)
                                               (/ (elt dimensions 1) 2)))
                                      (t dimensions))))
       :pose-stamped pose-stamped
       :color (desig-prop-value object 'desig-props:color))
      (when add
        (add-collision-object name pose-stamped t)))))

(defmethod register-collision-object ((name string)
                                      &key
                                        primitive-shapes
                                        mesh-shapes
                                        plane-shapes
                                        pose-stamped
                                        color)
  (let ((name (string-upcase (string name)))
        (obj (or (named-collision-object name)
                 (let ((obj-create
                         (make-instance 'collision-object
                                        :name name
                                        :primitive-shapes primitive-shapes
                                        :mesh-shapes mesh-shapes
                                        :plane-shapes plane-shapes
                                        :color color)))
                   (push obj-create *known-collision-objects*)
                   obj-create))))
    (when (and obj pose-stamped)
      (set-collision-object-pose name pose-stamped))))

(defun unregister-collision-object (name)
  (let ((name (string name)))
    (setf *known-collision-objects*
          (remove name *known-collision-objects*
                  :test (lambda (name object)
                          (equal name (slot-value object 'name)))))))

(defun named-collision-object (name)
  (let* ((name (string-upcase (string name)))
         (position (position name *known-collision-objects*
                             :test (lambda (name object)
                                     (equal name (slot-value object 'name))))))
    (when position
      (nth position *known-collision-objects*))))

(defun collision-object-pose (name)
  (let* ((col-obj (named-collision-object name)))
    (when col-obj
      (slot-value col-obj 'pose))))

(defun set-collision-object-pose (name pose-stamped)
  (let* ((col-obj (named-collision-object name)))
    (when col-obj
      (setf (slot-value col-obj 'pose) pose-stamped))))

(defun create-collision-object-message (name pose-stamped 
                                        &key
                                          primitive-shapes
                                          mesh-shapes
                                          plane-shapes)
  (let* ((name (string name)))
    (unless (or primitive-shapes mesh-shapes plane-shapes)
      (cpl:fail 'no-collision-shapes-defined))
    (flet* ((resolve-pose (pose-msg)
              (or pose-msg (tf:pose->msg pose-stamped)))
            (pose-present (object)
              (and (listp object) (cdr object)))
            (resolve-object (obj)
              (or (and (listp obj) (car obj)) obj))
            (prepare-shapes (shapes)
              (map 'vector 'identity (mapcar #'resolve-object shapes)))
            (prepare-poses (poses)
              (map 'vector #'resolve-pose (mapcar #'pose-present poses))))
      (let* ((obj-msg (roslisp:make-msg
                       "moveit_msgs/CollisionObject"
                       (stamp header) (tf:stamp pose-stamped)
                       (frame_id header) (tf:frame-id pose-stamped)
                       id name
                       operation (roslisp-msg-protocol:symbol-code
                                  'moveit_msgs-msg:collisionobject
                                  :add)
                       primitives (prepare-shapes primitive-shapes)
                       primitive_poses (prepare-poses primitive-shapes)
                       meshes (prepare-shapes mesh-shapes)
                       mesh_poses (prepare-poses mesh-shapes)
                       planes (prepare-shapes plane-shapes)
                       plane_poses (prepare-poses plane-shapes))))
        obj-msg))))

(defun make-object-color (id color)
  (let ((col-vec (case (intern (string-upcase (string color)) 'desig-props)
                   (blue (vector 0.0 0.0 1.0))
                   (red (vector 1.0 0.0 0.0))
                   (green (vector 0.0 1.0 0.0))
                   (yellow (vector 1.0 1.0 0.0))
                   (black (vector 0.0 0.0 0.0))
                   (white (vector 1.0 1.0 1.0))
                   (t (vector 1.0 0.0 1.0)))))
    (roslisp:make-message
     "moveit_msgs/ObjectColor"
     id id
     (r color) (elt col-vec 0)
     (g color) (elt col-vec 1)
     (b color) (elt col-vec 2)
     (a color) 1.0)))

(defun add-collision-object (name &optional pose-stamped
                                    (fixed-map-odomcombined nil))
  (let* ((name (string name))
         (col-obj (named-collision-object name))
         (pose-stamped (or pose-stamped
                           (collision-object-pose name))))
    (when (and col-obj pose-stamped)
      (setf (slot-value col-obj 'pose) pose-stamped)
      (let ((primitive-shapes (slot-value col-obj 'primitive-shapes))
            (mesh-shapes (slot-value col-obj 'mesh-shapes))
            (plane-shapes (slot-value col-obj 'plane-shapes))
            (color (slot-value col-obj 'color)))
        (declare (ignorable color))
        (let* ((obj-msg (roslisp:modify-message-copy
                         (create-collision-object-message
                          name pose-stamped
                          :primitive-shapes primitive-shapes
                          :mesh-shapes mesh-shapes
                          :plane-shapes plane-shapes)
                         operation (roslisp-msg-protocol:symbol-code
                                    'moveit_msgs-msg:collisionobject
                                    :add)))
               (world-msg (roslisp:make-msg
                           "moveit_msgs/PlanningSceneWorld"
                           collision_objects (vector obj-msg)))
               (scene-msg (roslisp:make-msg
                           "moveit_msgs/PlanningScene"
                           world world-msg
                           ;object_colors (vector (make-object-color name color))
                           is_diff t)))
          (prog1 (roslisp:publish *planning-scene-publisher* scene-msg)
            (roslisp:ros-info
             (moveit)
             "Added `~a' to environment server." name)
            (publish-object-colors)))))))

(defun remove-collision-object (name)
  (let* ((name (string name))
         (col-obj (named-collision-object name)))
    (when col-obj
      (let* ((obj-msg (roslisp:make-msg
                       "moveit_msgs/CollisionObject"
                       id name
                       operation (roslisp-msg-protocol:symbol-code
                                  'moveit_msgs-msg:collisionobject
                                  :remove)))
             (world-msg (roslisp:make-msg
                         "moveit_msgs/PlanningSceneWorld"
                         collision_objects (vector obj-msg)))
             (scene-msg (roslisp:make-msg
                         "moveit_msgs/PlanningScene"
                         world world-msg
                         is_diff t)))
        (prog1 (roslisp:publish *planning-scene-publisher* scene-msg)
          (roslisp:ros-info
           (moveit)
           "Removed `~a' from environment server." name))))))

(defmacro without-collision-objects (object-names &body body)
  `(unwind-protect
        (progn
          (dolist (object-name ,object-names)
            (remove-collision-object object-name))
          ,@body)
     (dolist (object-name ,object-names)
       (add-collision-object object-name))))

(defmacro without-collision-object (object-name &body body)
  `(without-collision-objects (list ,object-name) ,@body))

(defun clear-collision-objects ()
  (loop for col-obj in *known-collision-objects*
        do (remove-collision-object (slot-value col-obj 'name))))

(defun clear-collision-environment ()
  (clear-collision-objects)
  (setf *known-collision-objects* nil)
  (roslisp:ros-info (moveit) "Cleared collision environment."))

(defun attach-collision-object-to-link (name target-link
                                        &key current-pose-stamped touch-links)
  (let* ((name (string name))
         (col-obj (named-collision-object name))
         (current-pose-stamped (or current-pose-stamped
                                   (collision-object-pose name))))
    (when (and col-obj current-pose-stamped)
      (let ((primitive-shapes (slot-value col-obj 'primitive-shapes))
            (mesh-shapes (slot-value col-obj 'mesh-shapes))
            (plane-shapes (slot-value col-obj 'plane-shapes)))
        (roslisp:ros-info (moveit) "Transforming link from ~a into ~a"
                          (tf:frame-id current-pose-stamped)
                          target-link)
        (let* ((pose-in-link
                 (cl-tf2:ensure-pose-stamped-transformed
                  *tf2* current-pose-stamped target-link
                  :use-current-ros-time t))
               (obj-msg-plain (create-collision-object-message
                               name pose-in-link
                               :primitive-shapes primitive-shapes
                               :mesh-shapes mesh-shapes
                               :plane-shapes plane-shapes))
               (obj-msg (roslisp:modify-message-copy
                         obj-msg-plain
                         operation (roslisp-msg-protocol:symbol-code
                                    'moveit_msgs-msg:collisionobject
                                    :remove)))
               (attach-msg (roslisp:make-msg
                            "moveit_msgs/AttachedCollisionObject"
                            link_name target-link
                            object (roslisp:modify-message-copy
                                    obj-msg-plain
                                    operation (roslisp-msg-protocol:symbol-code
                                               'moveit_msgs-msg:collisionobject
                                               :add)
                                    id name)
                            touch_links (concatenate
                                         'vector
                                         (list target-link) touch-links)
                            weight 1.0))
               (world-msg (roslisp:make-msg
                           "moveit_msgs/PlanningSceneWorld"
                           collision_objects (vector obj-msg)))
               (scene-msg (roslisp:make-msg
                           "moveit_msgs/PlanningScene"
                           world world-msg
                           (attached_collision_objects robot_state) (vector
                                                                     attach-msg)
                           is_diff t)))
          (roslisp:publish *planning-scene-publisher* scene-msg)
          (set-collision-object-pose name pose-in-link)
          (roslisp:ros-info
           (moveit)
           "Attached collision object `~a' to link `~a'."
           name target-link))))))

(defun detach-collision-object-from-link (name target-link
                                          &key current-pose-stamped)
  (let* ((name (string name))
         (col-obj (named-collision-object name))
         (current-pose-stamped (or current-pose-stamped
                                   (collision-object-pose name))))
    (when (and col-obj current-pose-stamped)
      (let ((primitive-shapes (slot-value col-obj 'primitive-shapes))
            (mesh-shapes (slot-value col-obj 'mesh-shapes))
            (plane-shapes (slot-value col-obj 'plane-shapes))
            (time (roslisp:ros-time)))
        (unless (tf:wait-for-transform
                 *tf*
                 :timeout 5.0
                 :time time
                 :source-frame (tf:frame-id current-pose-stamped)
                 :target-frame target-link)
          (cpl:fail 'pose-not-transformable-into-link))
        (let* ((pose-in-link (cl-tf2:ensure-pose-stamped-transformed
                              *tf2* (tf:copy-pose-stamped
                                     current-pose-stamped
                                     :stamp time)
                              target-link :use-current-ros-time t))
               (obj-msg-plain (create-collision-object-message
                               name pose-in-link
                               :primitive-shapes primitive-shapes
                               :mesh-shapes mesh-shapes
                               :plane-shapes plane-shapes))
               (obj-msg (roslisp:modify-message-copy
                         obj-msg-plain
                         operation (roslisp-msg-protocol:symbol-code
                                    'moveit_msgs-msg:collisionobject
                                    :add)))
               (attach-msg (roslisp:make-msg
                            "moveit_msgs/AttachedCollisionObject"
                            object (roslisp:modify-message-copy
                                    obj-msg-plain
                                    operation (roslisp-msg-protocol:symbol-code
                                               'moveit_msgs-msg:collisionobject
                                               :remove)
                                    id name)))
               (world-msg (roslisp:make-msg
                           "moveit_msgs/PlanningSceneWorld"
                           collision_objects (vector obj-msg)))
               (scene-msg (roslisp:make-msg
                           "moveit_msgs/PlanningScene"
                           world world-msg
                           (attached_collision_objects robot_state) (vector
                                                                     attach-msg)
                           is_diff t)))
          (roslisp:publish *planning-scene-publisher* scene-msg)
          (set-collision-object-pose name pose-in-link)
          (roslisp:ros-info
           (moveit)
           "Detaching collision object `~a' from link `~a'."
           name (tf:frame-id current-pose-stamped)))))))

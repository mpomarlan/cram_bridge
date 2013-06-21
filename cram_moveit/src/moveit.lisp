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

(defvar *move-group-action-client* nil
  "Action client for the MoveGroup action.")
(defvar *planning-scene-publisher* nil
  "Publisher handle for the planning scene topic.")
(defvar *attached-object-publisher* nil
  "Publisher handle for attaching and detaching collicion objects through /attached_collision_object.")
(defvar *joint-states-fluent* nil
  "Fluent that keeps track of whether joint states were received or now.")
(defvar *joint-states* nil
  "List of current joint states as published by /joint_states.")
(defvar *joint-states-subscriber* nil
  "Subscriber to /joint_states.")

(defun init-moveit-bridge ()
  "Sets up the basic action client communication handles for the
  MoveIt! framework and registers known conditions."
  (register-known-moveit-errors)
  (setf *move-group-action-client*
        (actionlib:make-action-client
         "move_group" "moveit_msgs/MoveGroupAction"))
  (setf *collision-object-publisher*
        (roslisp:advertise
         "/collision_object"
         "moveit_msgs/CollisionObject" :latch t))
  (setf *planning-scene-publisher*
        (roslisp:advertise
         "/planning_scene"
         "moveit_msgs/PlanningScene" :latch t))
  (setf *attached-object-publisher*
        (roslisp:advertise
         "/attached_collision_object"
         "moveit_msgs/AttachedCollisionObject" :latch t))
  (setf *joint-states-fluent*
        (cram-language:make-fluent :name "joint-state-tracker"))
  (setf *joint-states-subscriber*
        (roslisp:subscribe "/joint_states"
                           "sensor_msgs/JointState"
                           #'joint-states-callback)))

(defun joint-states-callback (msg)
  (roslisp:with-fields (name position) msg
    (setf
     *joint-states*
     (loop for i from 0 below (length name)
           for n = (elt name i)
           for p = (elt position i)
           collect (cons n p)))))

(register-ros-init-function init-moveit-bridge)

(defun joint-states ()
  *joint-states*)

(defun copy-physical-joint-states (joint-names)
  (let* ((joint-states (joint-states))
         (relevant-states (loop for joint-name on joint-names
                                for position = (position
                                                joint-name joint-states
                                                :test (lambda (name state)
                                                        (equal
                                                         (car name)
                                                         (car state))))
                               when position
                                 collect (elt joint-states position))))
    (set-planning-robot-state relevant-states)))

(defun set-planning-robot-state (joint-states)
  (let* ((rstate-msg (roslisp:make-msg
                      "moveit_msgs/RobotState"
                      (name joint_state) (map 'vector (lambda (joint-state)
                                                        (car joint-state))
                                              joint-states)
                      (position joint_state) (map 'vector (lambda (joint-state)
                                                            (cdr joint-state))
                                                  joint-states)
                      (velocity joint_state) (map 'vector
                                                  (lambda (joint-state)
                                                    (declare
                                                     (ignore joint-state))
                                                    0.0)
                                                  joint-states)
                      (effort joint_state) (map 'vector
                                                (lambda (joint-state)
                                                  (declare
                                                   (ignore joint-state))
                                                  0.0)
                                                joint-states)))
         (scene-msg (roslisp:make-msg
                     "moveit_msgs/PlanningScene"
                     robot_state rstate-msg
                     is_diff t)))
    (prog1 (roslisp:publish *planning-scene-publisher* scene-msg)
      (roslisp:ros-info
       (moveit)
       "Setting robot planning state."))))

(defun move-link-pose (link-name planning-group pose-stamped
                       &key allowed-collision-objects
                         plan-only)
  "Calls the MoveIt! MoveGroup action. The link identified by
  `link-name' is tried to be positioned in the pose given by
  `pose-stamped'. Returns `T' on success and `nil' on failure, in
  which case a failure condition is signalled, based on the error code
  returned by the MoveIt! service (as defined in
  moveit_msgs/MoveItErrorCodes)."
  (declare (ignore allowed-collision-objects)) ;; Implement this.
  ;; NOTE(winkler): Since MoveIt! crashes once it receives a frame-id
  ;; which includes the "/" character at the beginning, we change the
  ;; frame-id here just in case.
  (let ((pose-stamped (tf:pose->pose-stamped
                       (let ((str (tf:frame-id pose-stamped)))
                         (cond ((string= (elt str 0) "/")
                                (subseq str 1))
                               (t str)))
                       (tf:stamp pose-stamped)
                       pose-stamped)))
    (let ((mpreq (make-message
                  "moveit_msgs/MotionPlanRequest"
                  :group_name planning-group
                  :num_planning_attempts 1
                  :allowed_planning_time 5.0
                  :goal_constraints
                  (vector
                   (make-message
                    "moveit_msgs/Constraints"
                    :position_constraints
                    (vector
                     (make-message
                      "moveit_msgs/PositionConstraint"
                      :weight 1.0
                      :link_name link-name
                      :header
                      (make-message
                       "std_msgs/Header"
                       :frame_id (tf:frame-id pose-stamped)
                       :stamp (tf:stamp pose-stamped))
                      :constraint_region
                      (make-message
                       "moveit_msgs/BoundingVolume"
                       :primitives
                       (vector
                        (make-message
                         "shape_msgs/SolidPrimitive"
                         :type (roslisp-msg-protocol:symbol-code
                                'shape_msgs-msg:solidprimitive :box)
                         :dimensions (vector 0.001 0.001 0.001)))
                       :primitive_poses
                       (vector
                        (tf:pose->msg pose-stamped)))))
                    :orientation_constraints
                    (vector
                     (make-message
                      "moveit_msgs/OrientationConstraint"
                      :weight 1.0
                      :link_name link-name
                      :header
                      (make-message
                       "std_msgs/Header"
                       :frame_id (tf:frame-id pose-stamped)
                       :stamp (tf:stamp pose-stamped))
                      :orientation
                      (make-message
                       "geometry_msgs/Quaternion"
                       :x (tf:x (tf:orientation pose-stamped))
                       :y (tf:y (tf:orientation pose-stamped))
                       :z (tf:z (tf:orientation pose-stamped))
                       :w (tf:w (tf:orientation pose-stamped)))
                      :absolute_x_axis_tolerance 0.001
                      :absolute_y_axis_tolerance 0.001
                      :absolute_z_axis_tolerance 0.001))))))
          (options (make-message
                    "moveit_msgs/PlanningOptions"
                    :planning_scene_diff
                    (make-message "moveit_msgs/PlanningScene"
                                  :is_diff t)
                    :plan_only plan-only)))
      (cond ((actionlib:connected-to-server *move-group-action-client*)
             (cpl:with-failure-handling
                 ((actionlib:server-lost (f)
                    (declare (ignore f))
                    (error 'planning-failed)))
               (let ((result (actionlib:call-goal
                              *move-group-action-client*
                              (actionlib:make-action-goal
                                  *move-group-action-client*
                                :request mpreq
                                :planning_options options))))
                 (roslisp:with-fields (error_code
                                       trajectory_start
                                       planned_trajectory) result
                   (roslisp:with-fields (val) error_code
                     (unless (eql val (roslisp-msg-protocol:symbol-code
                                       'moveit_msgs-msg:moveiterrorcodes
                                       :success))
                       (signal-moveit-error val))
                     (values trajectory_start planned_trajectory))))))
            (t (error 'actionlib:server-lost))))))

(defun plan-link-movement (link-name planning-group pose-stamped
                                     &key allowed-collision-objects)
  (cpl:with-failure-handling
      ((moveit:no-ik-solution (f)
         (declare (ignore f))
         (return))
       (moveit:planning-failed (f)
         (declare (ignore f))
         (return))
       (moveit:goal-violates-path-constraints (f)
         (declare (ignore f))
         (return))
       (moveit:invalid-goal-constraints (f)
         (declare (ignore f))
         (return))
       (moveit:goal-in-collision (f)
         (declare (ignore f))
         (return)))
    (moveit:move-link-pose
     link-name
     planning-group pose-stamped
     :allowed-collision-objects
     allowed-collision-objects
     :plan-only t)))

(defun pose-distance (link-frame pose-stamped)
  (tf:wait-for-transform
   *tf* :timeout 5.0
        :time (tf:stamp pose-stamped)
        :source-frame (tf:frame-id pose-stamped)
        :target-frame link-frame)
  (let ((transformed-pose-stamped (tf:transform-pose
                                   *tf* :pose pose-stamped
                                        :target-frame link-frame)))
    (tf:v-dist (tf:make-identity-vector) (tf:origin
                                          transformed-pose-stamped))))

(defun create-collision-object-message (name pose-stamped 
                                        &key
                                          primitive-shapes
                                          mesh-shapes
                                          plane-shapes)
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
      obj-msg)))

(defun add-collision-object (name pose-stamped
                             &key
                               primitive-shapes
                               mesh-shapes
                               plane-shapes)
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
                     is_diff t)))
    (prog1 (roslisp:publish *planning-scene-publisher* scene-msg)
      (roslisp:ros-info
       (moveit)
       "Added collision object to environment server."))))

(defun remove-collision-object (name)
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
       "Removed collision object `~a' from environment server." name))))

(defun clear-collision-objects ()
  (remove-collision-object "all"))

(defun attach-collision-object-to-link (name target-link current-pose-stamped
                                        &key
                                          touch-links
                                          primitive-shapes
                                          mesh-shapes
                                          plane-shapes)
  (unless (tf:wait-for-transform
           *tf*
           :timeout 5.0
           :time (tf:stamp current-pose-stamped)
           :source-frame (tf:frame-id current-pose-stamped)
           :target-frame target-link)
    (cpl:fail 'pose-not-transformable-into-link))
  (let* ((pose-in-link (tf:transform-pose
                        *tf*
                        :pose current-pose-stamped
                        :target-frame target-link))
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
                     (attached_collision_objects robot_state) (vector attach-msg)
                     is_diff t)))
    (prog1 (roslisp:publish *planning-scene-publisher* scene-msg)
      (roslisp:ros-info
       (moveit)
       "Attached collision object `~a' to link `~a'." name target-link))))

(defun detach-collision-object-from-link (name target-link current-pose-stamped
                                          &key
                                            primitive-shapes
                                            mesh-shapes
                                            plane-shapes)
  (unless (tf:wait-for-transform
           *tf*
           :timeout 5.0
           :time (tf:stamp current-pose-stamped)
           :source-frame (tf:frame-id current-pose-stamped)
           :target-frame target-link)
    (cpl:fail 'pose-not-transformable-into-link))
  (let* ((pose-in-link (tf:transform-pose
                        *tf*
                        :pose current-pose-stamped
                        :target-frame target-link))
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
                     (attached_collision_objects robot_state) (vector attach-msg)
                     is_diff t)))
    (prog1 (roslisp:publish *planning-scene-publisher* scene-msg)
      (roslisp:ros-info
       (moveit)
       "Detaching collision object `~a' from link `~a'."
       name (tf:frame-id current-pose-stamped)))))

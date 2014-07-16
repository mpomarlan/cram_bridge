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

(defun init-moveit-bridge ()
  "Sets up the basic action client communication handles for the
MoveIt! framework and registers known conditions."
  (register-known-moveit-errors)
  (setf *planning-scene-publisher*
        (roslisp:advertise
         "/planning_scene"
         "moveit_msgs/PlanningScene" :latch t))
  (setf *robot-state-display-publisher*
        (roslisp:advertise
         *robot-state-display-topic*
         "moveit_msgs/DisplayRobotState" :latch t))
  (setf *joint-states-fluent*
        (cram-language:make-fluent :name "joint-state-tracker" :allow-tracing nil))
  (setf *joint-states-subscriber*
        (roslisp:subscribe "/joint_states"
                           "sensor_msgs/JointState"
                           #'joint-states-callback))
  (setf *tf2* (make-instance 'cl-tf2:buffer-client))
  (connect-action-client)
  (ubiquitous-utilities:register-collision-object-registration-function
   #'register-collision-object)
  (ubiquitous-utilities:register-collision-object-adding-function
   #'add-collision-object)
  (ubiquitous-utilities:register-pose-transform-function
   #'ensure-pose-stamped-transformed))

(roslisp-utilities:register-ros-init-function init-moveit-bridge)

(defun move-link-pose (link-name planning-group pose-stamped
                       &key allowed-collision-objects
                         plan-only touch-links
                         ignore-collisions
                         start-state
                         collidable-objects)
  "Calls the MoveIt! MoveGroup action. The link identified by
  `link-name' is tried to be positioned in the pose given by
  `pose-stamped'. Returns `T' on success and `nil' on failure, in
  which case a failure condition is signalled, based on the error code
  returned by the MoveIt! service (as defined in
  moveit_msgs/MoveItErrorCodes)."
  ;; NOTE(winkler): Since MoveIt! crashes once it receives a frame-id
  ;; which includes the "/" character at the beginning, we change the
  ;; frame-id here just in case.
  (ros-info (moveit) "Move link: ~a (~a, ignore collisions: ~a)"
            link-name planning-group ignore-collisions)
  (let* ((start-state (or start-state (make-message "moveit_msgs/RobotState")))
         (allowed-collision-objects
           (mapcar #'string
                   (cond (ignore-collisions
                          (loop for obj in *known-collision-objects*
                                collect (slot-value obj 'name)))
                         (t allowed-collision-objects))))
         (touch-links
           (mapcar (lambda (x) (string x)) touch-links))
         (link-names (cond ((listp link-name) link-name)
                           (t `(,link-name))))
         (poses-stamped (mapcar
                         (lambda (pose-stamped)
                           (tf:pose->pose-stamped
                            (unslash-frame (tf:frame-id pose-stamped))
                            (tf:stamp pose-stamped)
                            pose-stamped))
                         (cond ((listp pose-stamped) pose-stamped)
                               (t `(,pose-stamped))))))
    (let* ((mpreq (make-message
                   "moveit_msgs/MotionPlanRequest"
                   :group_name planning-group
                   :num_planning_attempts 3
                   :allowed_planning_time 5.0
                   :goal_constraints
                   (make-pose-goal-constraints link-names poses-stamped)))
           (options
             (make-message
              "moveit_msgs/PlanningOptions"
              :planning_scene_diff
              (make-message
               "moveit_msgs/PlanningScene"
               :is_diff t
               :allowed_collision_matrix
               (relative-collision-matrix-msg
                `(,touch-links
                  ,collidable-objects)
                `(,allowed-collision-objects
                  ,(when collidable-objects (loop for obj in *known-collision-objects*
                                                  collect (slot-value obj 'name))))
                `(t t))
               :robot_state start-state)
              :plan_only plan-only
              :replan t
              :replan_attempts 3)))
      (cpl:with-failure-handling
          ((invalid-motion-plan (f)
             (declare (ignore f))
             (ros-warn (moveit) "Invalid motion plan. Rethrowing.")
             (error 'manipulation-failed)))
        (roslisp:with-fields (error_code
                              trajectory_start
                              planned_trajectory)
            (send-action *move-group-action-client*
                         :request mpreq
                         :planning_options options)
          (roslisp:with-fields (val) error_code
            (signal-moveit-error val))
          (values trajectory_start planned_trajectory))))))

(defun plan-base-movement (x y theta)
  (move-link-joint-states
   "base"
   (list (cons "virtual_joint/theta" theta)
         (cons "virtual_joint/x" x)
         (cons "virtual_joint/y" y))))

(defun move-link-joint-states (planning-group joint-states)
  (let* ((mpreq (make-message
                 "moveit_msgs/MotionPlanRequest"
                 :group_name planning-group
                 :num_planning_attempts 1
                 :allowed_planning_time 1
                 :goal_constraints
                 (vector
                  (make-message
                   "moveit_msgs/Constraints"
                   :joint_constraints
                   (map 'vector (lambda (joint-state)
                                  (make-message
                                   "moveit_msgs/JointConstraint"
                                   :joint_name (car joint-state)
                                   :position (cdr joint-state)
                                   :tolerance_below 0.1
                                   :tolerance_above 0.1
                                   :weight 1.0))
                        joint-states)))))
         (options
           (make-message
            "moveit_msgs/PlanningOptions"
            :planning_scene_diff
            (make-message
             "moveit_msgs/PlanningScene"
             :is_diff t)
            :plan_only t)))
    (cpl:with-failure-handling
        ((invalid-motion-plan (f)
           (declare (ignore f))
           (ros-warn (moveit) "Invalid motion plan. Rethrowing.")
           (error 'manipulation-failed)))
      (let ((result (send-action *move-group-action-client*
                                 :request mpreq
                                 :planning_options options)))
        (cond (result
               (roslisp:with-fields (error_code
                                     trajectory_start
                                     planned_trajectory)
                   result
                 (roslisp:with-fields (val) error_code
                   (unless
                       (eql val
                            (roslisp-msg-protocol:symbol-code
                             'moveit_msgs-msg:moveiterrorcodes
                             :success))
                     (signal-moveit-error val))
                   (values
                    trajectory_start planned_trajectory))))
              (t (ros-error (moveit)
                            "Empty actionlib response.")
                 (connect-action-client)
                 (error 'planning-failed)))))))

(defun execute-trajectory (trajectory &key (wait-for-execution t))
  (let ((result (call-service "/execute_kinematic_path"
                              'moveit_msgs-srv:ExecuteKnownTrajectory
                              :trajectory trajectory
                              :wait_for_execution wait-for-execution)))
    (roslisp:with-fields (error_code) result
      (roslisp:with-fields (val) error_code
        (unless (eql val (roslisp-msg-protocol:symbol-code
                          'moveit_msgs-msg:moveiterrorcodes
                          :success))
          (signal-moveit-error val))))
    t))

(defun compute-ik (link-name planning-group pose-stamped &key robot-state)
  "Computes an inverse kinematics solution (if possible) of the given
kinematics goal (given the link name `link-name' to position, the
`planning-group' to take into consideration, and the final goal pose
`pose-stamped' for the given link). Returns the final joint state on
success, and `nil' otherwise."
  (let ((result (roslisp:call-service
                 "/compute_ik"
                 "moveit_msgs/GetPositionIK"
                 :ik_request
                 (make-message
                  "moveit_msgs/PositionIKRequest"
                  :group_name planning-group
                  :ik_link_names (vector link-name)
                  :pose_stamped_vector (vector (tf:pose-stamped->msg
                                                pose-stamped))
                  :robot_state (or robot-state
                                   (make-message "moveit_msgs/RobotState"))))))
    (roslisp:with-fields (solution error_code) result
      (roslisp:with-fields (val) error_code
        (unless (eql val (roslisp-msg-protocol:symbol-code
                          'moveit_msgs-msg:moveiterrorcodes
                          :success))
          (signal-moveit-error val))
        solution))))

(defun plan-link-movements (link-name planning-group poses-stamped
                            &key allowed-collision-objects
                              touch-links default-collision-entries
                              ignore-collisions
                              destination-validity-only)
  (every (lambda (pose-stamped)
           (plan-link-movement
            link-name planning-group pose-stamped
            :allowed-collision-objects allowed-collision-objects
            :touch-links touch-links
            :ignore-collisions ignore-collisions
            :destination-validity-only destination-validity-only))
         poses-stamped))

(defun plan-link-movement (link-name planning-group pose-stamped
                           &key allowed-collision-objects
                             touch-links
                             ignore-collisions
                             destination-validity-only
                             highlight-links)
  "Plans the movement of link `link-name' to given goal-pose
`pose-stamped', taking the planning group `planning-group' into
consideration. Returns the proposed trajectory, and final joint state
on finding a valid motion plan for the given configuration from the
current configuration. If the flag `destination-validity-only' is set,
only the final state (but not the motion path trajectory in between)
is returned. Setting this flag also speeds up the process very much,
as only the final configuration IK is generated."
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
       (moveit:invalid-motion-plan (f)
         (declare (ignore f))
         (return))
       (moveit:goal-in-collision (f)
         (declare (ignore f))
         (return)))
    (cond (destination-validity-only
           (let ((ik (compute-ik link-name planning-group pose-stamped)))
             (when (and ik highlight-links)
               (display-robot-state ik :highlight highlight-links))
             ik))
          (t (moveit:move-link-pose
              link-name
              planning-group pose-stamped
              :allowed-collision-objects allowed-collision-objects
              :plan-only t
              :touch-links touch-links
              :ignore-collisions ignore-collisions)))))

(defun make-pose-goal-constraints (link-names poses-stamped
                                   &key (tolerance-radius 0.01))
  (map 'vector
       (lambda (link-name pose-stamped)
         (make-message
          "moveit_msgs/Constraints"
          :position_constraints
          (vector
           (make-message
            "moveit_msgs/PositionConstraint"
            :weight 1.0
            :link_name link-name
            :header (make-message
                     "std_msgs/Header"
                     :frame_id (tf:frame-id pose-stamped)
                     :stamp (tf:stamp pose-stamped))
            :constraint_region
            (make-message
             "moveit_msgs/BoundingVolume"
             :primitives (vector
                          (make-message
                           "shape_msgs/SolidPrimitive"
                           :type (roslisp-msg-protocol:symbol-code
                                  'shape_msgs-msg:solidprimitive :sphere)
                           :dimensions (vector tolerance-radius)))
             :primitive_poses (vector (tf:pose->msg pose-stamped)))))
          :orientation_constraints
          (vector
           (make-message
            "moveit_msgs/OrientationConstraint"
            :weight 1.0
            :link_name link-name
            :header (make-message
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
            :absolute_x_axis_tolerance tolerance-radius
            :absolute_y_axis_tolerance tolerance-radius
            :absolute_z_axis_tolerance tolerance-radius))))
       link-names poses-stamped))

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

(defvar *move-group-action-client* nil)
(defvar *joint-states-fluent* nil
  "Fluent that keeps track of whether joint states were received or now.")
(defvar *joint-states* nil
  "List of current joint states as published by /joint_states.")
(defvar *joint-states-subscriber* nil
  "Subscriber to /joint_states.")
(defvar *tf-subscriber* nil)

(defparameter *scene-msg* nil)

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
  (roslisp:publish *planning-scene-publisher* *scene-msg*)
  (setf *joint-states-fluent*
        (cram-language:make-fluent :name "joint-state-tracker"))
  (setf *joint-states-subscriber*
        (roslisp:subscribe "/joint_states"
                           "sensor_msgs/JointState"
                           #'joint-states-callback))
  (connect-action-client :reconnect t :reconnection-tries 5))

(defun connect-action-client (&key (reconnect t) reconnection-tries)
  (cond (reconnect
         (cond (reconnection-tries
                (loop for i from 1 to reconnection-tries
                      while (or (not *move-group-action-client*)
                                (not (actionlib:wait-for-server
                                      *move-group-action-client*
                                      3.0)))
                      do (setf *move-group-action-client*
                               (actionlib:make-action-client
                                "/move_group"
                                "moveit_msgs/MoveGroupAction"))))
               (t (loop while (or (not *move-group-action-client*)
                                  (not (actionlib:wait-for-server
                                        *move-group-action-client*
                                        3.0)))
                        do (setf *move-group-action-client*
                                 (actionlib:make-action-client
                                  "/move_group"
                                  "moveit_msgs/MoveGroupAction"))))))
        (t (setf *move-group-action-client*
                 (actionlib:make-action-client
                  "/move_group" "moveit_msgs/MoveGroupAction")))))

(defun joint-states-callback (msg)
  (roslisp:with-fields (name position) msg
    (setf
     *joint-states*
     (loop for i from 0 below (length name)
           for n = (elt name i)
           for p = (elt position i)
           collect (cons n p)))))

(roslisp-utilities:register-ros-init-function init-moveit-bridge)

(defun joint-states (&optional names)
  (cond (names
         (mapcar (lambda (name)
                   (find name *joint-states*
                         :test (lambda (name joint-state)
                                 (string= name (car joint-state)))))
                 names))
        (t *joint-states*)))

(defun get-joint-value (name)
  (let* ((joint-states (joint-states))
         (joint-state
           (nth (position name joint-states
                          :test (lambda (name state)
                                  (equal name (car state))))
                joint-states)))
    (cdr joint-state)))

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

(defun set-planning-robot-pose (pose-stamped)
  (let* ((rpose
           (let ((pose-map
                   (ensure-pose-stamped-transformed
                    pose-stamped "/odom_combined"
                    :ros-time t)))
             (vector
              (roslisp:make-msg
               "geometry_msgs/TransformStamped"
               (stamp header) (tf:stamp pose-map)
               (frame_id header) "/odom_combined"
               (child_frame_id) "odom_combined"
               (x translation transform) (tf:x (tf:origin pose-map))
               (y translation transform) (tf:y (tf:origin pose-map))
               (z translation transform) (tf:z (tf:origin pose-map))
               (x rotation transform) (tf:x (tf:orientation pose-map))
               (y rotation transform) (tf:y (tf:orientation pose-map))
               (z rotation transform) (tf:z (tf:orientation pose-map))
               (w rotation transform) (tf:w (tf:orientation pose-map))))))
         (scene-msg (roslisp:make-msg
                     "moveit_msgs/PlanningScene"
                     fixed_frame_transforms rpose
                     is_diff t)))
    (setf *scene-msg* scene-msg)
    (when *planning-scene-publisher*
      (prog1
          (roslisp:publish *planning-scene-publisher* scene-msg)
        (roslisp:ros-info (moveit) "Setting robot planning pose.")))))

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
    (setf *scene-msg* scene-msg)
    (when *planning-scene-publisher*
      (prog1
          (roslisp:publish *planning-scene-publisher* scene-msg)
        (roslisp:ros-info (moveit) "Setting robot planning state.")))))

(defun move-link-pose (link-name planning-group pose-stamped
                       &key allowed-collision-objects
                         plan-only touch-links
                         default-collision-entries
                         ignore-collisions
                         corridor
                         (wait-until-finished t)
                         (tolerance-sphere-radius 0.01)
                         (fixed-map-odomcombined t))
  "Calls the MoveIt! MoveGroup action. The link identified by
  `link-name' is tried to be positioned in the pose given by
  `pose-stamped'. Returns `T' on success and `nil' on failure, in
  which case a failure condition is signalled, based on the error code
  returned by the MoveIt! service (as defined in
  moveit_msgs/MoveItErrorCodes)."
  ;; NOTE(winkler): Since MoveIt! crashes once it receives a frame-id
  ;; which includes the "/" character at the beginning, we change the
  ;; frame-id here just in case.
  (let ((allowed-collision-objects
          (cond (ignore-collisions
                 (loop for obj in *known-collision-objects*
                       collect (slot-value obj 'name)))
                (t (mapcar (lambda (x)
                             (string x))
                           allowed-collision-objects))))
        (touch-links
          (mapcar (lambda (x) (string x)) touch-links))
        (pose-stamped (tf:pose->pose-stamped
                       (let ((str (tf:frame-id pose-stamped)))
                         (cond ((string= (elt str 0) "/")
                                (subseq str 1))
                               (t str)))
                       (tf:stamp pose-stamped)
                       pose-stamped))
        (path-constraints
          (cond (corridor (make-message
                           "moveit_msgs/Constraints"
                           :position_constraints
                           (vector
                            (make-message
                             "moveit_msgs/PositionConstraint"
                             :header
                             (make-message
                              "std_msgs/Header"
                              :frame_id (tf:frame-id pose-stamped)
                              :stamp (tf:stamp pose-stamped))
                             :link_name link-name
                             :constraint_region corridor
                             :weight 0.1))))
                (t (make-message "moveit_msgs/Constraints")))))
    (let* ((mpreq (make-message
                   "moveit_msgs/MotionPlanRequest"
                   :group_name planning-group
                   :num_planning_attempts 10
                   :allowed_planning_time 10
                   :workspace_parameters
                   (make-message
                    "moveit_msgs/WorkspaceParameters"
                    :header
                    (make-message
                     "std_msgs/Header"
                     :frame_id link-name
                     :stamp (tf:stamp pose-stamped))
                    :min_corner
                    (make-message
                     "geometry_msgs/Vector3"
                        :x -2
                        :y -2
                        :z -2)
                    :max_corner
                    (make-message
                     "geometry_msgs/Vector3"
                        :x 2
                        :y 2
                        :z 2))
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
                                 'shape_msgs-msg:solidprimitive :sphere)
                          :dimensions (vector tolerance-sphere-radius)))
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
                       :absolute_x_axis_tolerance 0.01
                       :absolute_y_axis_tolerance 0.01
                       :absolute_z_axis_tolerance 0.01))))
                   :path_constraints path-constraints))
                   ;:trajectory_constraints
                   ;(make-message
                   ; "moveit_msgs/TrajectoryConstraints"
                   ; :constraints (vector path-constraints))))
           (touch-links-concat (concatenate 'vector allowed-collision-objects
                                            touch-links))
           (options
             (make-message
              "moveit_msgs/PlanningOptions"
              :planning_scene_diff
              (make-message
               "moveit_msgs/PlanningScene"
               :is_diff t
               :allowed_collision_matrix
               (make-message
                "moveit_msgs/AllowedCollisionMatrix"
                :entry_names touch-links-concat
                :entry_values
                ;; NOTE(winkler): This loop is kind
                ;; of ugly. There sure must be a
                ;; more elegant solution to this.
                (map 'vector
                     (lambda (x1)
                       (declare (ignore x1))
                       (make-message
                        "moveit_msgs/AllowedCollisionEntry"
                        :enabled
                        (map 'vector
                             (lambda (x2)
                               (declare (ignore x2))
                               t)
                             touch-links-concat)))
                     touch-links-concat)
                :default_entry_names (map
                                      'vector (lambda (x)
                                                (car x))
                                      default-collision-entries)
                :default_entry_values (map
                                       'vector (lambda (x)
                                                 (cdr x))
                                       default-collision-entries))
               :fixed_frame_transforms
               (cond (fixed-map-odomcombined
                      (vector (transform-stamped->msg
                               (ensure-transform-available
                                "/map" "/odom_combined"))))
                     (t (vector))))
              :plan_only plan-only
              :replan t
              :replan_attempts 3)))
      (cond ((actionlib:wait-for-server *move-group-action-client* 5.0)
             (cpl:with-failure-handling
                 ((actionlib:server-lost (f)
                    (declare (ignore f))
                    (ros-error (moveit)
                               "Lost actionlib connection while executing.")
                    (connect-action-client)
                    (cpl:fail 'planning-failed))
                  (invalid-motion-plan (f)
                    (declare (ignore f))
                    (ros-warn (moveit) "Invalid motion plan. Rethrowing.")
                    (error 'manipulation-failed)))
               (let ((goal (actionlib:make-action-goal
                               *move-group-action-client*
                             :request mpreq
                             :planning_options options)))
                 (cond (wait-until-finished
                        (let ((result (actionlib:call-goal
                                       *move-group-action-client*
                                       goal
                                       :result-timeout 20.0 :timeout 20.0)))
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
                                   (error 'actionlib:server-lost)))))
                       (t (actionlib:send-goal *move-group-action-client*
                                               goal))))))
            (t (ros-error (moveit) "Lost actionlib connection while waiting.")
               (connect-action-client)
               (error 'planning-failed))))))

(defun move-link-joint-states (link-name planning-group joint-states)
  (let* ((mpreq (make-message
                 "moveit_msgs/MotionPlanRequest"
                 :group_name planning-group
                 :num_planning_attempts 10
                 :allowed_planning_time 10
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
                                   :tolerance_below 0.0
                                   :tolerance_above 0.0
                                   :weight 1.0))
                        joint-states))))))
    (cond ((actionlib:wait-for-server *move-group-action-client* 5.0)
           (cpl:with-failure-handling
               ((actionlib:server-lost (f)
                  (declare (ignore f))
                  (ros-error (moveit)
                             "Lost actionlib connection while executing.")
                  (connect-action-client)
                  (cpl:fail 'planning-failed))
                (invalid-motion-plan (f)
                  (declare (ignore f))
                  (ros-warn (moveit) "Invalid motion plan. Rethrowing.")
                  (error 'manipulation-failed)))
             (let ((goal (actionlib:make-action-goal
                             *move-group-action-client*
                           :request mpreq)))
               (let ((result (actionlib:call-goal
                              *move-group-action-client*
                              goal
                              :result-timeout 20.0 :timeout 20.0)))
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
                          (error 'actionlib:server-lost)))))))
          (t (ros-error (moveit) "Lost actionlib connection while waiting.")
             (connect-action-client)
             (error 'planning-failed)))))

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

(defun compute-ik (link-name planning-group pose-stamped)
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
                                                pose-stamped))))))
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
            :default-collision-entries default-collision-entries
            :ignore-collisions ignore-collisions
            :destination-validity-only destination-validity-only))
         poses-stamped))

(defun plan-link-movement (link-name planning-group pose-stamped
                           &key allowed-collision-objects
                             touch-links default-collision-entries
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
              :allowed-collision-objects
              allowed-collision-objects
              :plan-only t
              :touch-links touch-links
              :default-collision-entries default-collision-entries
              :ignore-collisions ignore-collisions)))))

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

(in-package :cpl-impl)

(defmethod hook-before-task-execution :around (name log-parameters)
  (beliefstate:start-node name log-parameters 1))

(defmethod hook-after-task-execution :around (id)
  (beliefstate:stop-node id))

(defmethod hook-before-named-top-level :around (name)
  (beliefstate:start-node name `() 1))

(defmethod hook-after-named-top-level :around (id)
  (beliefstate:stop-node id))

(defmethod hook-on-fail :around (condition)
  (beliefstate:add-failure-to-active-node condition))


(in-package :plan-lib)

(defmethod hook-before-performing-action-designator :around (designator matching-process-modules)
  (prog1
      (beliefstate:start-node
       "PERFORM-ACTION-DESIGNATOR"
       (list
        (list 'description
              (write-to-string
               (desig:description designator)))
        (list 'matching-process-modules
              (write-to-string
               matching-process-modules)))
       2)
    (beliefstate:add-designator-to-active-node designator)))

(defmethod hook-after-performing-action-designator :around (id success)
  (declare (ignore success))
  ;; NOTE(winkler): Success is not used at the moment. It would be
  ;; nice to have an additional service, saying "append data to
  ;; current task". The success would the go there.
  (beliefstate:stop-node id))

(defmethod hook-with-designator :around (designator)
  (beliefstate:add-designator-to-active-node
   designator))


(in-package :desig)

(defmethod hook-equate-designators :around (desig-child desig-parent)
  (beliefstate:equate-designators
   desig-child desig-parent))


(in-package :crs)

;; Switch off prolog logging for now
;; (defmethod hook-before-proving-prolog :around (query binds)
;;   (beliefstate:start-node "PROLOG"
;;                           (list (list 'query (write-to-string query))
;;                                 (list 'bindings (write-to-string binds)))
;;                           3))

;; (defmethod hook-after-proving-prolog :around (id success)
;;   (beliefstate:stop-node id :success success))


(in-package :cram-uima)

(defmethod hook-before-uima-request :around (designator-request)
  (let ((id (beliefstate:start-node
             "UIMA-PERCEIVE"
             (cram-designators:description designator-request) 2)))
    (beliefstate:add-designator-to-active-node designator-request
                                               :annotation "perception-request")
    id))

(defmethod hook-after-uima-request :around (id result)
  (dolist (desig result)
    (beliefstate:add-object-to-active-node
     desig :annotation "perception-result"))
  (beliefstate:add-topic-image-to-active-node *kinect-topic-rgb*)
  (beliefstate:stop-node id :success (not (eql result nil))))


(in-package :pr2-manipulation-process-module)

(defmethod hook-before-grasp :around (obj-desig)
  (prog1
      (beliefstate:start-node
       "GRASP"
       `() 2)
    (beliefstate:add-topic-image-to-active-node *kinect-topic-rgb*)
    (beliefstate:add-designator-to-active-node obj-desig
                                               :annotation "object-acted-on")))

(defmethod hook-after-grasp :around (id success)
  (progn
    (beliefstate:add-topic-image-to-active-node *kinect-topic-rgb*)
    (beliefstate:stop-node id :success success)))

(defmethod hook-before-putdown :around (obj-desig loc-desig)
  (prog1
      (beliefstate:start-node
       "PUTDOWN"
       `() 2)
    (beliefstate:add-topic-image-to-active-node *kinect-topic-rgb*)
    (beliefstate:add-designator-to-active-node
     obj-desig :annotation "object-acted-on")
    (beliefstate:add-designator-to-active-node
     loc-desig :annotation "putdown-location")))

(defmethod hook-after-putdown :around (id success)
  (progn
    (beliefstate:add-topic-image-to-active-node *kinect-topic-rgb*)
    (beliefstate:stop-node id :success success)))

(defmethod hook-before-move-arm :around (side pose-stamped
                                 planning-group ignore-collisions)
  (prog1
      (beliefstate:start-node
       "VOLUNTARY-BODY-MOVEMENT"
       `() 2)
    (beliefstate:add-designator-to-active-node
     (make-designator
      'cram-designators:action
      `((link ,side)
        (goal-pose ,pose-stamped)
        (planning-group ,planning-group)
        (ignore-collisions ,(cond (ignore-collisions 1)
                                  (t 0)))))
     :annotation "voluntary-movement-details")))

(defmethod hook-after-move-arm :around (id success)
  (beliefstate:stop-node id :success success))

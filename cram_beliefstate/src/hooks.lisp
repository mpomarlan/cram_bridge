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

(in-package :beliefstate)

(defvar *enable-prolog-logging* nil)

(defmethod cram-language::on-open-gripper cram-beliefstate (side max-effort position)
  (let ((id (beliefstate:start-node "OPEN-GRIPPER" `() 2)))
    (beliefstate:add-designator-to-node
     (make-designator
      'cram-designators:action
      `((side ,side)
        (max-effort ,max-effort)
        (position ,position)))
     id :annotation "gripper-command-details")
    (beliefstate:stop-node id)))

(defmethod cram-language::on-close-gripper cram-beliefstate (side max-effort position)
  (let ((id (beliefstate:start-node "CLOSE-GRIPPER" `() 2)))
    (beliefstate:add-designator-to-node
     (make-designator
      'cram-designators:action
      `((side ,side)
        (max-effort ,max-effort)
        (position ,position)))
     id :annotation "gripper-command-details")
    (beliefstate:stop-node id)))

(defmethod cram-language::on-invert-decision-tree cram-beliefstate
    (target-result features)
  (let ((result (alter-node `((features ,features)
                              (target-result ,target-result))
                            :mode :service
                            :command "invert-decision-tree")))
    result))

(defmethod cram-language::on-annotate-parameters cram-beliefstate (parameters)
  (add-designator-to-active-node
   (make-designator 'object `(,parameters))
   :annotation "parameter-annotation"))

(defmethod cram-language::on-predict cram-beliefstate (active-parameters
                                                         requested-features)
  (let ((result (alter-node `((active-features ,active-parameters)
                              (requested-features ,requested-features))
                            :mode :service
                            :command "predict"))
        (result-hash-table (make-hash-table)))
    (when result
      (loop for item in (description (first result))
            do (destructuring-bind (label value) item
                 (let ((interned-label (intern (symbol-name label)
                                               'desig-props)))
                   (setf
                    (gethash interned-label result-hash-table)
                    (cond ((eql interned-label 'desig-props::failures)
                           (cond ((listp value)
                                  (let ((fail-hash (make-hash-table)))
                                    (loop for item in value
                                          do (destructuring-bind (fail val) item
                                               (setf (gethash
                                                      (intern
                                                       (symbol-name fail)
                                                       'cram-plan-failures)
                                                      fail-hash)
                                                     val)))
                                    fail-hash))
                                 (t (make-hash-table))))
                          (t value)))))))
    result-hash-table))

(defmethod cram-language::on-load-model cram-beliefstate (file)
  "Load model for prediction."
  (alter-node `((command "load-model")
                (type "task-tree")
                (file ,(string file)))
              :mode :service
              :command "load-model"))

(defmethod cram-language::on-load-decision-tree cram-beliefstate (file)
  "Load decision tree for prediction."
  (alter-node `((command "load-model")
                (type "decision-tree")
                (file ,(string file)))
              :mode :service
              :command "load-model"))

(defmethod cram-language::on-publishing-collision-object
    cram-beliefstate (object obj-name)
  ;; (with-slots ((pose sem-map-coll-env::pose)
  ;;              (type sem-map-coll-env::type)
  ;;              (index sem-map-coll-env::index)
  ;;              (name sem-map-coll-env::name)
  ;;              (dimensions sem-map-coll-env::dimensions)) object
  ;;   (let* ((surfaces
  ;;            `(,(cons "kitchen_island" "HTTP://IAS.CS.TUM.EDU/KB/KNOWROB.OWL#KITCHEN_ISLAND_COUNTER_TOP-0")
  ;;              ,(cons "kitchen_sink_block" "HTTP://IAS.CS.TUM.EDU/KB/KNOWROB.OWL#KITCHEN_SINK_BLOCK_COUNTER_TOP-0")))
  ;;          (entry (find obj-name surfaces :test (lambda (x y)
  ;;                                                 (string= x (cdr y))))))
  ;;     (when entry
  ;;       (let ((offset-pose (tf:make-pose (tf:v+ (tf:origin pose)
  ;;                                               (tf:make-3d-vector 0 0 0.01))
  ;;                                        (tf:orientation pose))))
  ;;         (beliefstate:register-interactive-object
  ;;          (car entry) 'box offset-pose
  ;;          (vector (sem-map-coll-env::x dimensions)
  ;;                  (sem-map-coll-env::y dimensions)
  ;;                  (sem-map-coll-env::z dimensions))
  ;;          `((examine-tabletop ((label ,(concatenate 'string
  ;;                                                    "Examine countertop '"
  ;;                                                    (car entry)
  ;;                                                    "'"))
  ;;                               (parameter ,(car entry))))))))))
  )

(defmethod cram-language::on-preparing-performing-action-designator
    cram-beliefstate (designator matching-process-modules)
  (let ((id (beliefstate:start-node
             "PERFORM-ACTION-DESIGNATOR"
             (list
              (list 'description
                    (desig:description designator))
              (list 'matching-process-modules
                    matching-process-modules))
             2)))
    (beliefstate:add-designator-to-node designator id)
    id))

(defmethod cram-language::on-finishing-performing-action-designator
    cram-beliefstate (id success)
  (declare (ignore success))
  ;; NOTE(winkler): Success is not used at the moment. It would be
  ;; nice to have an additional service, saying "append data to
  ;; current task". The success would the go there.
  (beliefstate:stop-node id))

(defmethod cram-language::on-with-designator
    cram-beliefstate (designator)
  (beliefstate:add-designator-to-active-node
   designator))

(defmethod cram-language::on-preparing-named-top-level cram-beliefstate (name)
  (let ((name (or name "ANONYMOUS-TOP-LEVEL")))
    (beliefstate:start-node name `() 1)))

(defmethod cram-language::on-finishing-named-top-level cram-beliefstate (id)
  (beliefstate:stop-node id))

(defmethod cram-language::on-preparing-task-execution cram-beliefstate (name log-parameters log-pattern)
  (let ((id (beliefstate:start-node name log-parameters 1)))
    (let* ((param-bindings
             (rest (assoc 'cram-language-implementation::parameters
                          log-pattern)))
           (bound-parameters
             (mapcar (lambda (bdg)
                       `(,(car bdg) ,(rest bdg)))
                     param-bindings))
           (body-code
             (rest (assoc 'cram-language-implementation::body
                          log-pattern)))
           (pattern
             (rest (assoc 'cram-language-implementation::pattern
                          log-pattern)))
           (name
             (rest (assoc 'cram-language-implementation::name
                          log-pattern)))
           (description
             (append
              (when name `((name ,name)))
              (when pattern `((pattern ,(map 'vector #'identity pattern))))
              (when bound-parameters
                `((parameters ,(map 'vector #'identity bound-parameters))))
              (when body-code
                `((body-code ,(map 'vector #'identity body-code)))))))
      (when description
        (beliefstate:add-designator-to-node
         (make-designator
          'action
          description)
         id :annotation "task-details")))
    id))

(defmethod cram-language::on-finishing-task-execution cram-beliefstate (id)
  (beliefstate:stop-node id))

(defmethod cram-language::on-fail cram-beliefstate (datum)
  (when (symbolp datum)
    (beliefstate:add-failure-to-active-node datum)))

(defmethod cram-utilities::on-equate-designators (successor parent)
  (beliefstate:equate-designators successor parent))

(defmethod cram-language::on-begin-execute-tag-task cram-beliefstate (task)
  (let ((name (slot-value task 'cpl-impl:name)))
    (let ((id (beliefstate:start-node "TAG" `() 2)))
      (beliefstate::annotate-parameter
       'tagName (write-to-string name))
      (beliefstate:add-designator-to-node
       (make-designator
        'cram-designators:action
        `((name ,name)))
       id :annotation "tag-details")
      id)))

(defmethod cram-language::on-finish-execute-tag-task cram-beliefstate (id)
  (beliefstate:stop-node id))

(defmethod cram-language::on-begin-move-head
    cram-beliefstate (pose-stamped)
  (let ((id (beliefstate:start-node
             "VOLUNTARY-BODY-MOVEMENT-HEAD"
             `() 2)))
    (beliefstate:add-designator-to-node
     (make-designator
      'cram-designators:action
      `((goal-pose ,pose-stamped)))
     id :annotation "voluntary-movement-details")
    id))

(defmethod cram-language::on-finish-move-head cram-beliefstate (id success)
  (beliefstate:stop-node id :success success))

(defmethod cram-language::on-begin-grasp cram-beliefstate (obj-desig)
  (let ((id (beliefstate:start-node "GRASP" `() 2)))
    (beliefstate:add-topic-image-to-active-node
     cram-beliefstate::*kinect-topic-rgb*)
    (beliefstate:add-designator-to-node
     obj-desig
     id :annotation "object-acted-on")
    id))

(defmethod cram-language::on-grasp-decisions-complete
    cram-beliefstate (log-id grasp-description)
  (let ((grasp (find 'desig-props::grasp
                     grasp-description :test (lambda (x y)
                                               (eql x (car y))))))
    (when grasp
      (let ((arm (find 'cram-manipulation-knowledge::arm
                       (second grasp)
                       :test (lambda (x y)
                               (eql x (car y))))))
        (when arm
          (annotate-parameter 'arm (second arm))))))
  (beliefstate:add-designator-to-node
   (make-designator 'cram-designators:action
                    grasp-description)
   log-id :annotation "grasp-details"))

(defmethod cram-language::on-finish-grasp cram-beliefstate (log-id success)
  (beliefstate:add-topic-image-to-active-node
   cram-beliefstate::*kinect-topic-rgb*)
  (beliefstate:stop-node log-id :success success))

(defmethod cram-language::on-begin-putdown cram-beliefstate (obj-desig loc-desig)
  (let ((id (beliefstate:start-node
             "PUTDOWN"
             `() 2)))
    (beliefstate:add-topic-image-to-active-node
     cram-beliefstate::*kinect-topic-rgb*)
    (beliefstate:add-designator-to-node
     obj-desig id :annotation "object-acted-on")
    (beliefstate:add-designator-to-node
     loc-desig id :annotation "putdown-location")
    id))

(defmethod cram-language::on-finish-putdown cram-beliefstate (log-id success)
  (beliefstate:add-topic-image-to-active-node
   cram-beliefstate::*kinect-topic-rgb*)
  (beliefstate:stop-node log-id :success success))

(defmethod cram-language::on-prepare-move-arm cram-beliefstate
    (side pose-stamped planning-group ignore-collisions)
  (let ((log-id
          (beliefstate:start-node
           "VOLUNTARY-BODY-MOVEMENT-ARMS"
           `() 2)))
    (beliefstate:add-designator-to-node
     (make-designator
      'cram-designators:action
      `((link ,side)
        (goal-pose ,pose-stamped)
        (planning-group ,planning-group)
        (ignore-collisions ,(cond (ignore-collisions 1)
                                  (t 0)))))
     log-id
     :annotation "voluntary-movement-details")
    log-id))

(defmethod cram-language::on-finish-move-arm cram-beliefstate (id success)
  (beliefstate:stop-node id :success success))

(defmethod cram-language::on-begin-motion-planning cram-beliefstate (link-name)
  (let ((id (beliefstate:start-node "MOTION-PLANNING" `() 2)))
    (beliefstate:add-designator-to-node
     (make-designator
      'cram-designators:action
      `((link-name ,link-name)))
     id :annotation "motion-planning-details")
    id))

(defmethod cram-language::on-finish-motion-planning cram-beliefstate (id)
  (beliefstate:stop-node id))

(defmethod cram-language::on-begin-motion-execution cram-beliefstate ()
  (let ((id (beliefstate:start-node "MOTION-EXECUTION" `() 2)))
    id))

(defmethod cram-language::on-finish-motion-execution cram-beliefstate (id)
  (beliefstate:stop-node id))

(defmethod cram-language::on-begin-find-objects cram-beliefstate ()
  (beliefstate:start-node "FIND-OBJECTS" `() 2))

(defmethod cram-language::on-finish-find-objects (id)
  (beliefstate:stop-node id))

(defmethod cram-language::on-prepare-perception-request cram-beliefstate (designator-request)
  (let ((id (beliefstate:start-node
             "UIMA-PERCEIVE"
             (cram-designators:description designator-request) 2)))
    (beliefstate:add-designator-to-node designator-request
                                        id :annotation "perception-request")
    id))

(defmethod cram-language::on-finish-perception-request cram-beliefstate (id designators-result)
  (dolist (desig designators-result)
    (beliefstate:add-object-to-node
     desig id :annotation "perception-result"))
  (beliefstate:add-topic-image-to-active-node cram-beliefstate::*kinect-topic-rgb*)
  (beliefstate:stop-node id :success (not (eql designators-result nil))))

(defmethod cram-language::on-with-failure-handling-begin cram-beliefstate (clauses)
  (let ((id (beliefstate:start-node "WITH-FAILURE-HANDLING" (mapcar (lambda (clause)
                                                                      `(clause ,clause))
                                                                    clauses) 2)))
    (beliefstate:add-designator-to-node
     (make-designator
      'cram-designators:action
      (mapcar (lambda (clause)
                `(clause ,clause))
              clauses))
     id :annotation "with-failure-handling-clauses")
    id))

(defmethod cram-language::on-with-failure-handling-handled cram-beliefstate (id)
  (catch-current-failure-with-active-node id))

(defmethod cram-language::on-with-failure-handling-rethrown cram-beliefstate (id)
  (rethrow-current-failure-with-active-node id))

(defmethod cram-language::on-with-failure-handling-end cram-beliefstate (id)
  (beliefstate:stop-node id))

(defmethod cram-language::on-with-policy-begin (name parameters)
  (let ((id (beliefstate:start-node "WITH-POLICY" (append `(policy ,name) parameters) 2)))
    (beliefstate:add-designator-to-node
     (make-designator
      'cram-designators:action
      `((name ,name)
        (parameters ,parameters)))
     id :annotation "with-policy-details")))

(defmethod cram-language::on-with-policy-end (id success)
  (beliefstate:stop-node id :success success))

(defmethod cram-utilities::on-prepare-json-prolog-prove cram-beliefstate (request)
  )

(defmethod cram-utilities::on-finish-json-prolog-prove cram-beliefstate (id)
  )

(defmethod cram-utilities::on-prepare-prolog-prove cram-beliefstate (query binds)
  (when *enable-prolog-logging*
    (let ((id (beliefstate:start-node "PROLOG" `() 3)))
      (beliefstate:add-designator-to-node
       ;; TODO(winkler): Properly log the query and bindings information
       ;; of the prolog operation.
       (make-designator
        'cram-designators:action
        `())
       id :annotation "prolog-details"))))

(defmethod cram-utilities::on-finish-prolog-prove cram-beliefstate (id success)
  (when *enable-prolog-logging*
    (beliefstate:stop-node id :success success)))

(defmethod cram-language::on-performing-object-grasp cram-beliefstate (object)
  (when object
    (let ((newest (newest-effective-designator object)))
      (when newest
        (let* ((at (desig-prop-value newest 'desig-props::at))
               (pose (desig-prop-value at 'desig-props::pose))
               (object-type (desig-prop-value newest 'desig-props::type)))
          (when pose
            (let* ((robot-pose-map
                     (cl-tf2:ensure-pose-stamped-transformed
                      *tf2* (tf:make-pose-stamped
                             "base_footprint" 0.0
                             (tf:make-identity-vector)
                             (tf:make-identity-rotation))
                      "map"))
                   (object-pose-map
                     (cl-tf2:ensure-pose-stamped-transformed
                      *tf2* pose "map"))
                   (distance-2d
                     (tf:v-dist (tf:make-3d-vector
                                 (tf:x (tf:origin robot-pose-map))
                                 (tf:y (tf:origin robot-pose-map))
                                 0.0)
                                (tf:make-3d-vector
                                 (tf:x (tf:origin object-pose-map))
                                 (tf:y (tf:origin object-pose-map))
                                 0.0)))
                   (angle-difference
                     (tf:angle-between-quaternions
                      (tf:orientation robot-pose-map)
                      (tf:orientation object-pose-map))))
              (annotate-parameter 'object-type object-type)
              (annotate-parameter 'distance-2d
                                  distance-2d)
              (annotate-parameter 'angle-difference-2d
                                  angle-difference))))))))

(defmethod cram-language::on-performing-object-putdown cram-beliefstate (object pose)
  (when object
    (let ((newest (newest-effective-designator object)))
      (when newest
        (let ((object-type (desig-prop-value
                            object 'desig-props::type)))
          (when pose
            (let* ((robot-pose-map
                     (cl-tf2:ensure-pose-stamped-transformed
                      *tf2* (tf:make-pose-stamped
                             "base_footprint" 0.0
                             (tf:make-identity-vector)
                             (tf:make-identity-rotation))
                      "map"))
                   (putdown-pose-map
                     (cl-tf2:ensure-pose-stamped-transformed
                      *tf2* pose "map"))
                   (distance-2d
                     (tf:v-dist (tf:make-3d-vector
                                 (tf:x (tf:origin robot-pose-map))
                                 (tf:y (tf:origin robot-pose-map))
                                 0.0)
                                (tf:make-3d-vector
                                 (tf:x (tf:origin putdown-pose-map))
                                 (tf:y (tf:origin putdown-pose-map))
                                 0.0)))
                   (angle-difference
                     (tf:angle-between-quaternions
                      (tf:orientation robot-pose-map)
                      (tf:orientation putdown-pose-map))))
              (annotate-parameter 'object-type object-type)
              (annotate-parameter 'distance-2d distance-2d)
              (annotate-parameter 'angle-difference-2d angle-difference))))))))

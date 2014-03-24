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

(defmethod sem-map-coll-env::on-publishing-collision-object
    cram-beliefstate (object obj-name)
  (with-slots ((pose sem-map-coll-env::pose)
               (type sem-map-coll-env::type)
               (index sem-map-coll-env::index)
               (name sem-map-coll-env::name)
               (dimensions sem-map-coll-env::dimensions)) object
    (let* ((surfaces
             `(,(cons "kitchen_island" "HTTP://IAS.CS.TUM.EDU/KB/KNOWROB.OWL#KITCHEN_ISLAND_COUNTER_TOP-0")
               ,(cons "kitchen_sink_block" "HTTP://IAS.CS.TUM.EDU/KB/KNOWROB.OWL#KITCHEN_SINK_BLOCK_COUNTER_TOP-0")))
           (entry (find obj-name surfaces :test (lambda (x y)
                                                  (string= x (cdr y))))))
      (when entry
        (let ((offset-pose (tf:make-pose (tf:v+ (tf:origin pose)
                                                (tf:make-3d-vector 0 0 0.01))
                                         (tf:orientation pose))))
          (beliefstate:register-interactive-object
           (car entry) 'box offset-pose
           (vector (sem-map-coll-env::x dimensions)
                   (sem-map-coll-env::y dimensions)
                   (sem-map-coll-env::z dimensions))
           `((examine-tabletop ((label ,(concatenate 'string
                                                     "Examine countertop '"
                                                     (car entry)
                                                     "'"))
                                (parameter ,(car entry)))))))))))

(defmethod plan-lib::on-preparing-performing-action-designator
    cram-beliefstate (designator matching-process-modules)
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

(defmethod plan-lib::on-finishing-performing-action-designator
    cram-beliefstate (id success)
  (declare (ignore success))
  ;; NOTE(winkler): Success is not used at the moment. It would be
  ;; nice to have an additional service, saying "append data to
  ;; current task". The success would the go there.
  (beliefstate:stop-node id))

(defmethod cram-language-designator-support::on-with-designator
    cram-beliefstate (designator)
  (beliefstate:add-designator-to-active-node
   designator))

(defmethod cpl-impl::on-preparing-named-top-level cram-beliefstate (name)
  (let ((name (or name "ANONYMOUS-TOP-LEVEL")))
    (beliefstate:start-node name `() 1)))

(defmethod cpl-impl::on-finishing-named-top-level cram-beliefstate (id)
  (beliefstate:stop-node id))

(defmethod cpl-impl::on-preparing-task-execution cram-beliefstate (name log-parameters)
  (prog1
      (beliefstate:start-node name log-parameters 1)
    (let ((goal-location (second
                          (find 'cram-plan-library::goal-location log-parameters
                                :test (lambda (x y)
                                        (eql x (first y)))))))
      (when goal-location
        (beliefstate:add-designator-to-active-node
         goal-location
         :annotation "goal-location")
        (beliefstate:add-designator-to-active-node
         (make-designator
          'cram-designators:location
          `((pose ,(reference goal-location))))
         :annotation "goal-pose")))))

(defmethod cpl-impl::on-finishing-task-execution cram-beliefstate (id)
  (beliefstate:stop-node id))

(defmethod cpl-impl::on-fail cram-beliefstate (datum)
  (when (symbolp datum)
    (beliefstate:add-failure-to-active-node datum)))

(defmethod desig::on-equate-designators (successor parent)
  (beliefstate:equate-designators successor parent))

(defmethod point-head-process-module::on-begin-move-head
    cram-beliefstate (pose-stamped)
  (prog1
      (beliefstate:start-node
       "VOLUNTARY-BODY-MOVEMENT-HEAD"
       `() 2)
    (beliefstate:add-designator-to-active-node
     (make-designator
      'cram-designators:action
      `((goal-pose ,pose-stamped)))
     :annotation "voluntary-movement-details")))

(defmethod point-head-process-module::on-finish-move-head cram-beliefstate (id success)
  (beliefstate:stop-node id :success success))

(defmethod pr2-manipulation-process-module::on-begin-grasp cram-beliefstate (obj-desig)
  (prog1
      (beliefstate:start-node "GRASP" `() 2)
    (beliefstate:add-topic-image-to-active-node
     cram-beliefstate::*kinect-topic-rgb*)
    (beliefstate:add-designator-to-active-node
     obj-desig
     :annotation "object-acted-on")))

(defmethod pr2-manipulation-process-module::on-grasp-decisions-complete
    cram-beliefstate (obj-name pregrasp-pose grasp-pose side object-pose)
  (beliefstate:add-designator-to-active-node
   (make-designator 'cram-designators:action
                    `((object-name ,obj-name)
                      (pregrasp-pose ,pregrasp-pose)
                      (grasp-pose ,grasp-pose)
                      (side ,side)
                      (object-pose ,object-pose)))
   :annotation "grasp-details"))

(defmethod pr2-manipulation-process-module::on-finish-grasp cram-beliefstate (log-id success)
  (beliefstate:add-topic-image-to-active-node
   cram-beliefstate::*kinect-topic-rgb*)
  (beliefstate:stop-node log-id :success success))

(defmethod pr2-manipulation-process-module::on-begin-putdown cram-beliefstate (obj-desig loc-desig)
  (prog1
      (beliefstate:start-node
       "PUTDOWN"
       `() 2)
    (beliefstate:add-topic-image-to-active-node
     cram-beliefstate::*kinect-topic-rgb*)
    (beliefstate:add-designator-to-active-node
     obj-desig :annotation "object-acted-on")
    (beliefstate:add-designator-to-active-node
     loc-desig :annotation "putdown-location")))

(defmethod pr2-manipulation-process-module::on-finish-putdown cram-beliefstate (log-id success)
  (beliefstate:add-topic-image-to-active-node
   cram-beliefstate::*kinect-topic-rgb*)
  (beliefstate:stop-node log-id :success success))

(defmethod pr2-manipulation-process-module::on-prepare-move-arm cram-beliefstate
    (side pose-stamped planning-group ignore-collisions)
  (prog1
      (beliefstate:start-node
       "VOLUNTARY-BODY-MOVEMENT-ARMS"
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

(defmethod pr2-manipulation-process-module::on-finish-move-arm cram-beliefstate (id success)
  (beliefstate:stop-node id :success success))

(defmethod cram-uima::on-prepare-request cram-beliefstate (designator-request)
  (let ((id (beliefstate:start-node
             "UIMA-PERCEIVE"
             (cram-designators:description designator-request) 2)))
    (beliefstate:add-designator-to-active-node designator-request
                                               :annotation "perception-request")
    id))

(defmethod cram-uima::on-finish-request cram-beliefstate (id result)
  (dolist (desig result)
    (beliefstate:add-object-to-active-node
     desig :annotation "perception-result"))
  (beliefstate:add-topic-image-to-active-node cram-beliefstate::*kinect-topic-rgb*)
  (beliefstate:stop-node id :success (not (eql result nil))))

(defmethod cpl-impl::on-with-failure-handling-begin cram-beliefstate (clauses)
  (prog1
      (beliefstate:start-node "WITH-FAILURE-HANDLING" (mapcar (lambda (clause)
                                                                `(clause ,clause))
                                                              clauses) 2)
    (beliefstate:add-designator-to-active-node
     (make-designator
      'cram-designators:action
      (mapcar (lambda (clause)
                `(clause ,clause))
              clauses))
     :annotation "with-failure-handling-clauses")))

(defmethod cpl-impl::on-with-failure-handling-handled cram-beliefstate (id)
  (catch-current-failure-with-active-node id))

(defmethod cpl-impl::on-with-failure-handling-end cram-beliefstate (id)
  (beliefstate:stop-node id))

(defmethod cpl::on-with-policy-begin (name parameters)
  (prog1
      (beliefstate:start-node "WITH-POLICY" (append `(policy ,name) parameters) 2)
    (beliefstate:add-designator-to-active-node
     (make-designator
      'cram-designators:action
      `((name ,name)
        (parameters ,parameters)))
     :annotation "with-policy-details")))

(defmethod cpl::on-with-policy-end (id success)
  (beliefstate:stop-node id :success success))

;; Switch off prolog logging for now
;; (defmethod cram-reasoning::on-prepare-prolog-prove cram-beliefstate (query binds)
;;   (beliefstate:start-node "PROLOG"
;;                           (list (list 'query (write-to-string query))
;;                                 (list 'bindings (write-to-string binds)))
;;                           3))

;; (defmethod cram-reasoning::on-finish-prolog-prove cram-beliefstate (id success)
;;   (beliefstate:stop-node id :success success))

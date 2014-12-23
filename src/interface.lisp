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
;;;     * Neither the name or Universitaet Bremen nor the names of its
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

(in-package :robosherlock-process-module)

(defclass perceived-object-data
    (desig:object-designator-data
     cram-manipulation-knowledge:object-shape-data-mixin)
  ((pose :reader pose :initarg :pose)
   (identifier :reader identifier :initarg :identifier)))

(defvar *ignored-bullet-objects* nil)

(defgeneric call-perception-routine (designator))
(defgeneric examine-perceived-object-designator
    (original-designator object-designator))
(defgeneric perceive-with-object-designator (designator &key target-frame))
(defgeneric designators-match (template subject))
(defgeneric filter-perceived-objects (template-designator perceived-objects))
(defgeneric get-volume-of-interest (object-designator))
(defgeneric make-uima-request-designator (&key object-designator))
(defgeneric perceive-object-designator (designator))

(defmacro mapcar-clean (function list &rest more-lists)
  "Automatically removes all `NIL' entries from a generated list after
performing a `mapcar'."
  (if more-lists
      `(remove-if #'not (mapcar ,function ,list ,more-lists))
      `(remove-if #'not (mapcar ,function ,list))))

(defun ignore-bullet-object (object-name)
  (setf *ignored-bullet-objects*
        (remove object-name *ignored-bullet-objects*))
  (push object-name *ignored-bullet-objects*))

(defmethod get-volume-of-interest ((object-designator object-designator))
  (let ((result (cut:lazy-car (crs:prolog `(obj-volume-of-interest
                                            ,object-designator ?p ?w ?h ?d)))))
    (when result
      (cut:with-vars-bound (?p ?w ?h ?d) result
        (when ?p
          (append
           `((desig-props::pose ,?p))
           (when ?w `((desig-props::w ,?w)))
           (when ?h `((desig-props::h ,?h)))
           (when ?d `((desig-props::d ,?d)))))))))

(defmethod make-uima-request-designator (&key object-designator)
  (let* ((volume-of-interest (get-volume-of-interest object-designator))
         (description (append
                       `((desig-props::object ,object-designator))
                       (when volume-of-interest
                         `((desig-props::voi ,volume-of-interest))))))
    (make-designator 'action description)))

(defmethod call-perception-routine ((object-designator object-designator))
  (let* ((request-designator (make-uima-request-designator
                              :object-designator object-designator))
         (uima-result-designators (uima:get-uima-result request-designator)))
    (labels ((sub-value (name sequence)
               (cadr (find name sequence :test (lambda (x y)
                                                 (eql x (car y)))))))
      (mapcar
       (lambda (uima-result-designator)
         (make-designator
          'object
          (remove-if
           #'not
           (mapcar (lambda (property)
                     (destructuring-bind (key value) property
                       (cond ((eql key 'color)
                              `(,key ,value))
                             ((eql key 'shape)
                              `(,key ,(intern (string-upcase value))))
                             ((eql key 'boundingbox)
                              `(,key
                                ,(mapcar (lambda (resolution-property)
                                           (destructuring-bind
                                               (key value) resolution-property
                                             (cond ((eql key 'dimensions-3d)
                                                    `(dimensions-3d
                                                      ,(vector
                                                        (sub-value 'width value)
                                                        (sub-value 'height value)
                                                        (sub-value 'depth value))))
                                                   (t resolution-property))))
                                         value)))
                             ((eql key 'segment)
                              `(,key
                                ,(mapcar (lambda (segment-property)
                                           (destructuring-bind
                                               (key value) segment-property
                                             (cond ((eql key 'dimensions-2d)
                                                    `(dimensions-2d
                                                      ,(vector
                                                        (sub-value 'width value)
                                                        (sub-value 'height value))))
                                                   (t segment-property))))
                                         value)))
                             (t `(,key ,value)))))
                   (description uima-result-designator))) uima-result-designator))
       uima-result-designators))))

(defmethod perceive-with-object-designator ((object-designator object-designator)
                                            &key (target-frame "map"))
  (let* ((log-id (first (cram-language::on-prepare-request
                         object-designator)))
         (perception-results
           (remove-if
            #'not
            (mapcar
             (lambda (perception-result)
               (cond ((desig-prop-value perception-result 'resolution)
                      perception-result)
                     (t (ros-warn
                         (robosherlock-pm)
                         "Object without resolution information. Dropping.")
                        nil)))
             (call-perception-routine object-designator))))
         (remove-properties `(pose pose-on-plane bb-pose resolution
                                   boundingbox at name)))
    (labels ((sub-value (name sequence)
               (cadr (find name sequence :test (lambda (x y)
                                                 (eql x (car y)))))))
      (let ((results
              (mapcar-clean
               (lambda (perception-result)
                 (let* ((new-description
                          (remove-if (lambda (x)
                                       (find (car x) remove-properties))
                                     (description perception-result)))
                        (pose (desig-prop-value perception-result 'pose))
                        (resolution (desig-prop-value perception-result
                                                      'resolution))
                        (id (sub-value 'objectid resolution))
                        (lastseen (sub-value 'lastseen resolution))
                        (boundingbox (desig-prop-value perception-result
                                                       'boundingbox))
                        (pose-bb (sub-value 'pose boundingbox))
                        (dimensions-3d (sub-value 'dimensions-3d boundingbox))
                        (additional-properties
                          (append
                           `((plane-distance ,(/ (elt dimensions-3d 2) 2)))
                           `((at ,(make-designator
                                   'location
                                   `((pose
                                      ,(cl-tf2:ensure-pose-stamped-transformed
                                        *tf2*
                                        (cond ((find 'flat (desig-prop-values
                                                            perception-result
                                                            'shape))
                                               pose)
                                              (t pose-bb))
                                        target-frame :use-current-ros-time t))))))
                           `((name ,(intern (concatenate 'string "OBJECT"
                                                         (write-to-string
                                                          (truncate id)))
                                            'desig-props)))
                           `((dimensions ,dimensions-3d)))))
                   (when (< lastseen 2.0d0)
                     (make-designator 'object (append new-description
                                                      additional-properties)
                                      perception-result))))
               perception-results)))
        (cram-language::on-finish-request log-id results)
        results))))

(defmethod examine-perceived-object-designator
    ((original-designator object-designator)
     (object-designator object-designator))
  "Enriches a perceived object designator `object-designator' with
additional information from the reasoning system. First, the type is
infered (if not set already), and then additional properties are
infered and appended to the designator's description."
  (let* ((object-description (description object-designator))
         (type (or (desig-prop-value object-designator 'type)
                   (cut:with-vars-bound (?type)
                       (first
                        (crs:prolog `(infer-object-property
                                      ,object-designator desig-props:type ?type)))
                     (format t "Type: ~a~%" ?type)
                     ?type)))
         (typed-object-designator
           (or (and (or (eql type '?type) (not type)) object-designator)
               (make-designator
                'object
                (append
                 (remove-if (lambda (x) (eql (car x) 'type))
                            object-description)
                 `((type ,type)))
                object-designator)))
         (handles
           (cut:force-ll (cut:lazy-mapcar
                          (lambda (bdgs)
                            (cut:with-vars-bound (?handles) bdgs
                              (loop for handle in ?handles
                                    collect `(desig-props::handle ,handle))))
                          (crs:prolog `(object-handle ,type ?handles)))))
         (new-properties
          (append
           (cut:force-ll (cut:lazy-mapcar
                          (lambda (bdgs)
                            (cut:with-vars-bound (?key ?value) bdgs
                              `(,?key ,?value)))
                          (crs:prolog `(infer-object-property
                                        ,typed-object-designator
                                        ?key ?value))))
           (first handles)))
         (refined-old
           (remove-if (lambda (x)
                        (find x new-properties
                              :test (lambda (x y)
                                      (eql (car x) (car y)))))
                      (description typed-object-designator))))
    (let* ((infered-description (append refined-old new-properties))
           (complete-description
             (let ((original-description (description original-designator)))
               (append infered-description
                       (mapcar-clean
                        (lambda (original-property)
                          (unless (find original-property
                                        infered-description
                                        :test (lambda (x y)
                                                (eql (car x) (car y))))))
                        original-description)))))
      (make-designator 'object complete-description object-designator))))

(defun remove-disappeared-objects (object-names)
  "Removes objects from the current bullet world. They are refered to
by the `name' property specified in their designator."
  (roslisp:ros-info (robosherlock-pm) "Removing objects: ~a~%"
                    (length object-names))
  (dolist (object-name object-names)
    (ros-info (robosherlock-pm) "Remove object: ~a~%" object-name)
    (crs:prolog `(and (btr:bullet-world ?w)
                      (btr:retract
                       (btr:object
                        ?w ,object-name))))
    (moveit:remove-collision-object object-name)))

(defun add-appeared-objects (objects)
  "Adds objects to the current bullet world. In the world, they then
consist of boxes of dimensions as specified in the `dimensions'
property in their designator."
  (ros-info (robosherlock-pm) "Adding objects: ~a~%"
                    (length objects))
  (dolist (object objects)
    (let ((pose (desig-prop-value
                 (desig-prop-value object 'at)
                 'pose))
          (dimensions (desig-prop-value object 'dimensions))
          (name (desig-prop-value object 'name)))
      (ros-info (robosherlock-pm) "Add object: ~a~%" name)
      (crs:prolog `(and (btr:bullet-world ?w)
                        (btr:assert
                         (btr:object
                          ?w btr:box ,name ,pose
                          :mass 0.1
                          :size ,(map 'list #'identity dimensions))))))
    (moveit:register-collision-object
     object :add t
     :pose-stamped (cl-tf2:ensure-pose-stamped-transformed
                    *tf2*
                    (desig-prop-value (desig-prop-value object 'at) 'pose)
                    "/map"))))

(defun update-objects (objects)
  "Updates objects' poses in the current bullet world based on the
`name', and the `pose' properties in their respective designator."
  (roslisp:ros-info (robosherlock-pm) "Updating objects: ~a~%"
                    (length objects))
  (dolist (object objects)
    (let ((pose (desig-prop-value
                 (desig-prop-value object 'at)
                 'pose))
          (name (desig-prop-value object 'name)))
      (ros-info (robosherlock-pm) "Update object: ~a~%" name)
      (crs:prolog `(and (btr:bullet-world ?w)
                        (btr:assert
                         (btr:object-pose
                          ?w ,name ,pose)))))
    (moveit:register-collision-object
     object :add t
     :pose-stamped (cl-tf2:ensure-pose-stamped-transformed
                    *tf2*
                    (desig-prop-value (desig-prop-value object 'at) 'pose)
                    "/map"))))

(defmethod designators-match ((template object-designator)
                              (subject object-designator))
  "Checks every property of the `template' object designator to be
present in the `subject' object designator. Returns `t' if all
properties in `template' are satisfied, `NIL' otherwise. Only checks
for: string, number, symbol. All other value types are ignored. This
way, reference and unknown object type comparisons are avoided."
  (loop for (key value) in (description template)
        for type-check-fnc = (cond ((stringp value) #'string=)
                                   ((numberp value) #'=)
                                   ((symbolp value) #'eql))
        for subject-values = (desig-prop-values subject key)
        when (and type-check-fnc subject-values)
          do (unless (find value subject-values :test type-check-fnc)
               (return nil))
        finally (return t))
  t)

(defmethod filter-perceived-objects ((template-designator object-designator)
                                     (perceived-objects list))
  "Filters out all object designator instances in the
`perceived-objects' list which descriptions don't match the properties
defined in `template-designator'."
  (mapcar-clean
   (lambda (perceived-object)
     (when (designators-match template-designator perceived-object)
       ;; The designator matches based on its description (if
       ;; any). Now see if it gets accepted based on external factors.
       perceived-object))
   perceived-objects))

(defmethod perceive-object-designator ((object-designator object-designator))
  "Triggers operation of the external perception system to find out
which objects are currently seen, and compares the result to the
internal beliefstate. Poses of known and visible objects are updated
in the beliefstate, new objects are added, and disappeared objects are
retracted from the internal representation. The parameter
`object-designator' describes the object to find."
  ;; Make sure that the current pose and everything is in the
  ;; beliefstate.
  (plan-knowledge:on-event (make-instance 'plan-knowledge:robot-state-changed))
  (let* ((perceived-object-designators
           (mapcar-clean
            (lambda (perceived-object)
              (unless (crs:prolog `(perceived-object-invalid
                                    ,perceived-object))
                perceived-object))
            (perceive-with-object-designator object-designator)))
         (perceived-object-names
           (mapcar (lambda (perceived-object-designator)
                     (desig-prop-value perceived-object-designator 'name))
                   perceived-object-designators))
         ;; Represents *all* objects present in the current bullet
         ;; world, except for the ones explicitly being ignored.
         (all-bullet-objects
           (cut:force-ll
            (cut:lazy-mapcar
             (lambda (bdgs)
               (cut:with-vars-bound (?o) bdgs
                 ?o))
             (crs:prolog `(and (btr:bullet-world ?w)
                               (btr:object ?w ?o)
                               (not (btr:robot ?o))
                               (not (member ?o ,*ignored-bullet-objects*)))))))
         ;; Identify all objects that are not present at all in the
         ;; bullet world, but are identified as being seen by the
         ;; perception system.
         (unknown-and-perceived
           (mapcar-clean
            (lambda (perceived-object-name)
              (when (not (find perceived-object-name all-bullet-objects))
                perceived-object-name))
            perceived-object-names)))
    (labels ((object-name->object (object-name)
               (find object-name perceived-object-designators
                     :test (lambda (name object)
                             (eql name (desig-prop-value object 'name)))))
             (object-names->objects (object-names)
               (mapcar #'object-name->object object-names)))
      ;; Add objects that were not present in the currently visible
      ;; bullet world portion, but were reported by the perception
      ;; system as being visible.
      (add-appeared-objects (object-names->objects unknown-and-perceived))
      (let* (;; Identify all objects that *should* be visible from the
             ;; robot's current view point.
             (should-be-visible
               (cut:force-ll
                (cut:lazy-mapcar 
                 (lambda (bdgs)
                   (cut:with-vars-bound (?o) bdgs
                     ?o))
                 (crs:prolog `(and (btr:bullet-world ?w)
                                   (btr:robot ?r)
                                   (member ?o ,all-bullet-objects)
                                   (btr:visible ?w ?r ?o))))))
             ;; Identify all objects that should be visible from the
             ;; bullet world, and are reported as being seen by the
             ;; perception system.
             (should-be-visible-and-perceived
               (mapcar-clean
                (lambda (perceived-object-name)
                  (find perceived-object-name should-be-visible))
                perceived-object-names)))
        ;; Update poses of already existing, currently visible objects
        ;; in the bullet world, based on information reported by the
        ;; perception system.
        (update-objects (object-names->objects should-be-visible-and-perceived))
        (let (;; Identify all objects that should be visible from the
              ;; bullet world, but are not reported as being seen by the
              ;; perception system.
              (should-be-visible-and-not-perceived
                (mapcar-clean
                 (lambda (should-be-visible-name)
                   (when (not (find should-be-visible-name
                                    perceived-object-names))
                     should-be-visible-name))
                 should-be-visible)))
          ;; Remove objects from the current bullet world that *should* be
          ;; visible based on visibility reasoning, but aren't according
          ;; to the perception system.
          (remove-disappeared-objects should-be-visible-and-not-perceived)
          ;; Filter perceived objects based on the description of the
          ;; request (template) designator.
          (filter-perceived-objects
           object-designator
           ;; Examine visible objects (new or updated) closer.
           (mapcar (lambda (examined-object-designator)
                     (let ((data (make-instance
                                  'perceived-object-data
                                  :identifier (desig-prop-value
                                               examined-object-designator 'name)
                                  :object-identifier (desig-prop-value
                                                      examined-object-designator 'name)
                                  :pose (desig-prop-value
                                         (desig-prop-value
                                          examined-object-designator 'desig-props::at)
                                         'desig-props::pose))))
                       (make-effective-designator
                        object-designator
                        :new-properties (description examined-object-designator)
                        :data-object data)))
                   (mapcar (lambda (perceived-object-designator)
                             (examine-perceived-object-designator
                              object-designator perceived-object-designator))
                           perceived-object-designators))))))))

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

(defparameter *proximity-threshold* 0.2)
(defparameter *object-marker-color* `(1.0 0.0 0.0 1.0))
(defparameter *object-reference-frame* "/map")

(defclass perceived-object (desig:object-designator-data
                            cram-manipulation-knowledge:object-shape-data-mixin)
  ((designator :reader designator :initarg :designator)
   (pose :reader pose :initarg :pose)
   (type :reader object-type :initarg :type)
   (identifier :reader identifier :initarg :identifier)
   (shape :reader shape :initarg :shape)
   (size :reader size :initarg :size)
   (color :reader color :initarg :color)
   (z-offset :reader z-offset :initarg :z-offset)
   (grasp-poses :reader grasp-poses :initarg :grasp-poses)
   (desig-props:dimensions :reader dimensions :initarg :dimensions)))

(defgeneric call-action (action &rest params))

(defmacro def-action-handler (name args &body body)
  (alexandria:with-gensyms (action-sym params)
    `(defmethod call-action ((,action-sym (eql ',name)) &rest ,params)
       (destructuring-bind ,args ,params ,@body))))

(defun bound-slot-value (object slot-name)
  (when (slot-boundp object slot-name)
    (slot-value object slot-name)))

(defun perceived-object->designator (perceived-object &key parent)
  (assert parent () "Parent not set!")
  (let* ((pose (slot-value perceived-object 'pose))
         (identifier (bound-slot-value perceived-object
                                       'identifier)))
    (let ((color (bound-slot-value perceived-object 'desig-props:color))
          (shape (bound-slot-value perceived-object 'desig-props:shape))
          (size (bound-slot-value perceived-object 'desig-props:size))
          (type (or (bound-slot-value perceived-object 'desig-props:type)
                    'desig-props:pancakemix));object))
          (side (desig-prop-value parent 'desig-props:side))
          (z-offset (bound-slot-value perceived-object 'desig-props:z-offset))
          (dimensions (bound-slot-value perceived-object
                                        'desig-props:dimensions))
          (grasp-poses (bound-slot-value perceived-object
                                         'desig-props:grasp-poses))
          (grasp-type (desig-prop-value parent 'desig-props:grasp-type)))
      (make-effective-designator
       parent
       :new-properties (desig:update-designator-properties
                        `(,@(when color `((desig-props:color ,color)))
                          ,@(when shape `((desig-props:shape ,shape)))
                          ,@(when size `((desig-props:size ,size)))
                          ,@(when grasp-type `((desig-props:grasp-type ,grasp-type)))
                          ,@(when type `((desig-props:type ,type)))
                          ,@(when side `((desig-props:side ,side)))
                          ,@(when z-offset `((desig-props:z-offset ,z-offset)))
                          ,@(when grasp-poses
                              (mapcar (lambda (gp)
                                        `(desig-props:grasp-pose ,gp))
                                      grasp-poses))
                          ,@(when dimensions `((desig-props:dimensions
                                                ,dimensions)))
                          (desig-props:at
                           ,(desig:make-designator
                             'desig:location `((desig-props:pose ,pose))))
                          ,@(when identifier `((desig-props:name ,identifier))))
                        (when parent (desig:properties parent)))
       :data-object perceived-object))))

(defun symbol-or-unknown (found-designator parent-designator symbol
                          &optional (default 'unknown))
  (intern (string-upcase
           (or (cram-designators:desig-prop-value
                found-designator symbol)
               (cram-designators:desig-prop-value
                parent-designator symbol)
               default))
          :desig-props))

(defun pose-pointing-away-from-base (object-pose)
  (let ((ref-frame "/base_link")
        (fin-frame *object-reference-frame*))
    (let* ((base-transform-map
             (moveit:ensure-transform-available
              ref-frame fin-frame))
           (base-pose-map (tf:make-pose-stamped
                           (tf:frame-id base-transform-map)
                           (tf:stamp base-transform-map)
                           (tf:translation base-transform-map)
                           (tf:rotation base-transform-map)))
           (object-pose-map
             (moveit:ensure-pose-stamped-transformed
              object-pose fin-frame))
           (origin1 (tf:origin base-pose-map))
           (origin2 (tf:origin object-pose-map))
           (p1 (tf:make-3d-vector (tf:x origin1) (tf:y origin1) 0.0))
           (p2 (tf:make-3d-vector (tf:x origin2) (tf:y origin2) 0.0))
           (angle (+ (* (signum (- (tf:y p2) (tf:y p1)))
                        (acos (/ (- (tf:x p2) (tf:x p1)) (tf:v-dist p1 p2))))
                     (/ pi -2))))
      (tf:make-pose-stamped fin-frame 0.0
                            (tf:origin object-pose-map)
                            (tf:euler->quaternion :az (+ angle (/ pi 2)))))))

(defun indexed-name (name index)
  (intern (concatenate 'string (symbol-name name) (write-to-string index)) 'desig-props))

(defun deduce-model-properties (prior-dimensions type)
  (cond (prior-dimensions
         (cond ((= (length prior-dimensions) 2)
                (cons 'desig-props:cylinder prior-dimensions))
               (t (cons 'desig-props:box prior-dimensions))))
        ((eql type 'desig-props:spatula)
         (cons 'desig-props:box (vector 0.32 0.08 0.06)))
        ((eql type 'desig-props:pancakemaker)
         (cons 'desig-props:cylinder (vector 0.07 0.15)))
        ((eql type 'desig-props:pancake)
         (cons 'desig-props:cylinder (vector 0.02 0.06)))
        (t (cons 'desig-props:box (vector 0.06 0.09 0.2)))))

(defun get-object-name (parent pose type &key (threshold *proximity-threshold*))
  (let ((pose-in-map
          (when pose (moveit:ensure-pose-stamped-transformed
                      pose "/map"))))
    (let ((near-objects
            (when pose-in-map
              (crs::force-ll
               (crs::lazy-mapcar
                (lambda (n)
                  (crs::var-value '?n n))
                (crs:prolog `(and (btr::bullet-world ?w)
                                  (btr::object ?w ?n)
                                  (not (equal ?n nil))
                                  (btr::object-pose ?w ?n ?p)
                                  (btr::poses-equal ?p ,pose-in-map (,threshold 6.4))))))))
          (all-objects
            (crs::force-ll
             (crs::lazy-mapcar
              (lambda (n)
                (crs::var-value '?n n))
              (crs:prolog
               `(and (btr::bullet-world ?w)
                     (btr::object ?w ?n)))))))
      (loop for i from 0 below (length all-objects)
            for temp-name = (indexed-name type i)
            when (find temp-name near-objects)
              do (return-from get-object-name temp-name))
      ;; Check if there are any *very close* objects that were not
      ;; detected as proper type yet. If that is the case, ignore this
      ;; object (as it most probably was perceived wrongly).
      (let ((very-close-objects
              (when pose-in-map
                (crs::force-ll
                 (crs::lazy-mapcar
                  (lambda (n)
                    (crs::var-value '?n n))
                  (crs:prolog `(and (btr::bullet-world ?w)
                                    (btr::object ?w ?n)
                                    (btr::object-pose ?w ?n ?p)
                                    (btr::poses-equal ?p ,pose-in-map (0.01 6.4))))))))
            (parent-name (desig-prop-value parent 'desig-props:name)))
        (let ((type (or (and
                         very-close-objects
                         (when (eql type 'desig-props:pancakemaker)
                           (dolist (close-object very-close-objects)
                             (crs:prolog `(and (btr:bullet-world ?w)
                                               (btr:retract ?w (btr:object ,close-object)))))
                           'desig-props:pancakemaker))
                        (and (not very-close-objects)
                             type))))
          (when type
            (cond (parent-name parent-name)
                  (t (loop for i from 0 to (length all-objects)
                           for temp-name = (indexed-name type i)
                           when (not (crs:prolog `(and (btr::bullet-world ?w)
                                                       (btr::object ?w ,temp-name))))
                             do (return-from get-object-name temp-name))))))))))

(defun replace-all (string part replacement &key (test #'char=))
  "Returns a new string in which all the occurences of `part' are
replaced with `replacement'."
  (with-output-to-string (out)
    (loop with part-length = (length part)
          for old-pos = 0 then (+ pos part-length)
          for pos = (search part string
                            :start2 old-pos
                            :test test)
          do (write-string string out
                           :start old-pos
                           :end (or pos (length string)))
          when pos do (write-string replacement out)
            while pos)))

(defun desig-msg-field->pose-stamped (desig &key
                                              (field 'desig-props:pose)
                                              (to-fixed-frame t)
                                              (reorient nil))
  (when (cram-designators:desig-prop-value desig field)
    (let* ((pose-stamped (tf:msg->pose-stamped
                          (cram-designators:desig-prop-value
                           desig field)))
           (maybe-transformed-pose
             (cond (to-fixed-frame
                    (moveit:ensure-pose-stamped-transformed pose-stamped *object-reference-frame*
                                                            :ros-time t))
                   (t pose-stamped))))
      (cond (reorient (pose-pointing-away-from-base maybe-transformed-pose))
            (t maybe-transformed-pose)))))

(defun get-perceived-objects (designator)
  ;; Wait for a few seconds as UIMA needs a moment to produce concise,
  ;; current results.
  (let ((waiting-time 4))
    (roslisp:ros-info () "Waiting for perception to settle on correct images (~a sec)." waiting-time)
    (cpl:sleep* waiting-time))
  (roslisp:ros-info () "Continuing.")
  (let ((objects (uima:get-uima-result designator))
        (default-object-type 'desig-props:pancakemix))
    (mapcar (lambda (object)
                     (let* ((type (or (intern
                                       (replace-all (string-upcase
                                                     (cram-designators:desig-prop-value
                                                      object 'desig-props:type)) "_" "")
                                       'desig-props)  default-object-type))
                            (dimensions (cdr (deduce-model-properties
                                              (cram-designators:desig-prop-value
                                               designator 'desig-props:dimensions)
                                              type)))
                            (needs-reorientation t)
                            (pose-on-plane (desig-msg-field->pose-stamped
                                            object
                                            :field 'desig-props:pose-on-plane
                                            :reorient needs-reorientation))
                            (pose (desig-msg-field->pose-stamped
                                   object
                                   :reorient needs-reorientation))
                            (name (get-object-name designator pose type))
                            (grasp-poses (cram-designators:desig-prop-values
                                          object 'desig-props:grasp-pose))
                            (z-offset (max (cond ((and pose pose-on-plane)
                                                  (tf:v-dist (tf:origin pose)
                                                             (tf:origin pose-on-plane)))
                                                 (t 0.0))
                                           0.0))
                            (shape (car (deduce-model-properties
                                         (cram-designators:desig-prop-value
                                          designator 'desig-props:dimensions)
                                         type)))
                            (color (symbol-or-unknown object designator
                                                      'desig-props:color 'desig-props:yellow))
                            (size (symbol-or-unknown object designator 'desig-props:size)))
                       (make-instance
                        'perceived-object
                        :object-identifier name
                        :identifier name
                        :pose pose
                        :type type
                        :shape shape
                        :color color
                        :size size
                        :z-offset z-offset
                        :dimensions dimensions
                        :grasp-poses grasp-poses)))
            objects)))

(defun find-object (designator)
  (let ((perceived-objs (get-perceived-objects designator))
        (registered-objects nil))
    (let* ((at (desig-prop-value designator 'desig-props:at))
           (pose-stamped (when at (desig-prop-value at 'desig-props:pose)))
           (pose-stamped-expected
             (when pose-stamped (moveit:ensure-pose-stamped-transformed
                                 pose-stamped *object-reference-frame* :ros-time t)))
           (valid-objects
             (loop for object in perceived-objs
                   for name = (slot-value object 'identifier)
                   for pose = (slot-value object 'pose)
                   for type = (slot-value object 'type)
                   for dimensions = (slot-value object 'desig-props:dimensions)
                   when (and name
                             pose
                             (or (not pose-stamped-expected)
                                 (<= (tf:v-dist
                                      (tf:make-3d-vector
                                       (tf:x (tf:origin pose-stamped-expected))
                                       (tf:y (tf:origin pose-stamped-expected)) 0)
                                      (tf:make-3d-vector
                                       (tf:x (tf:origin pose))
                                       (tf:y (tf:origin pose)) 0))
                                     *proximity-threshold*)))
                     collect
                     (progn
                       (push name registered-objects)
                       (register-object name type pose dimensions)
                       object))))
      (dolist (registered-object registered-objects)
        (crs:prolog `(and (btr:bullet-world ?w)
                          (btr:retract ?w (btr:object ,registered-object)))))
      valid-objects)))

(defun found-object-fits-prototype (found-object prototype-object)
  (let ((prot-type (desig-prop-value prototype-object 'desig-props:type))
        (obj-type (desig-prop-value found-object 'desig-props:type)))
    (or (not prot-type)
        (eql prot-type obj-type))))

(cut:define-hook on-prepare-request (designator-request))
(cut:define-hook on-finish-request (log-id designators-result))

(defun find-with-designator (designator)
  (let* ((log-id (first (on-prepare-request designator)))
         (found-objects
           (mapcar (lambda (perceived-object)
                     (perceived-object->designator perceived-object
                                                   :parent designator))
                   (find-object designator)))
         (fitting-objects
           (loop for object in found-objects
                 when (found-object-fits-prototype object designator)
                   collect object)))
    (on-finish-request log-id fitting-objects)
    fitting-objects))

(defun register-object (name type pose dimensions &key (z-offset 0.0))
  (declare (ignorable dimensions))
  (let ((pose (ubiquitous-utilities:transform-pose pose "/map")))
    (cond ((or (eql type 'desig-props:pancake)
               (eql type 'desig-props:pancakemaker))
           (crs:prolog `(and (btr:bullet-world ?w)
                             (btr:assert
                              (btr:object
                               ?w ,(cond ((eql type 'desig-props:pancake)
                                          'desig-props:pancake)
                                         (t 'btr::pancake-maker))
                               ,name ,(tf:copy-pose
                                       pose :origin (tf:v- (tf:origin pose)
                                                           (tf:make-3d-vector
                                                            0 0 z-offset)))
                               :size ,(cond ((eql type 'desig-props:pancakemaker)
                                             `(0.15 0.15 0.035))
                                            (t `(0.05 0.05 0.01)))
                               :mass 0.1)))))
          (t (crs:prolog `(and (btr:bullet-world ?w)
                               (btr:assert
                                (btr:object
                                 ?w btr:mesh ,name ,(tf:copy-pose
                                                     pose :origin (tf:v- (tf:origin pose)
                                                                         (tf:make-3d-vector
                                                                          0 0 (/ z-offset 2))))
                                 :mesh ,(cond ((eql type 'desig-props:pancakemix)
                                               'btr:mondamin)
                                              (t type))
                                 :mass 0.1))))))))

(defun perceive-with-designator (object-designator)
  (let* ((perceived-designators
           (find-with-designator (or (newest-effective-designator
                                      object-designator)
                                     object-designator))))
    (unless perceived-designators
      (cpl:fail 'cram-plan-failures:object-not-found
                :object-desig object-designator))
    (dolist (designator perceived-designators)
      (let* ((object-name (desig-prop-value designator 'desig-props:name))
             (object-dimensions (desig-prop-value designator 'desig-props:dimensions))
             (object-type (desig-prop-value designator 'desig-props:type))
             (object-at (desig-prop-value designator 'desig-props:at))
             (object-pose (desig-prop-value object-at 'desig-props:pose))
             (z-offset (desig-prop-value designator 'desig-props:z-offset)))
        ;; TODO(winkler): The object-pose here is in a wrong
        ;; frame. This changed due to the fact that bullet reasoning
        ;; expects '/map' to be the object reference frame, while
        ;; MoveIt! expects '/odom_combined'. Solving this next.
        (moveit:register-collision-object designator :add t :pose-stamped object-pose)
        (register-object object-name object-type object-pose object-dimensions :z-offset z-offset)
        (moveit:set-object-color object-name *object-marker-color*))
      (cram-plan-knowledge:on-event
       (make-instance
        'cram-plan-knowledge:object-perceived-event
        :perception-source :robosherlock :object-designator designator)))
    perceived-designators))

(def-action-handler perceive (object-designator)
  (ros-info (perception) "Perceiving object.")
  (perceive-with-designator object-designator))

(def-action-handler perceive-scene ()
  (ros-info (perception) "Perceiving scene.")
  (perceive-with-designator (make-designator 'object nil)))

(def-action-handler examine (object-designator)
  (declare (ignorable object-designator))
  (ros-info (perception) "Examining object."))

(def-process-module robosherlock-process-module (desig)
  (apply #'call-action (reference desig)))

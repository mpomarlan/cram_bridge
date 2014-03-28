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

(defparameter *proximity-threshold* 0.3)

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
                                       'desig-props:identifier)))
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

(defun elevate-if-necessary (pose-stamped supporting-z)
  (cond (supporting-z
         (let* ((pose-in-map (moveit:ensure-pose-stamped-transformed
                              pose-stamped "/map" :ros-time t))
                (pose-z (tf:z (tf:origin pose-in-map)))
                (diff-z (- pose-z supporting-z)))
           (values
            (tf:make-pose-stamped
             "/map" (tf:stamp pose-in-map)
             (tf:v+ (tf:origin pose-in-map)
                    (tf:make-3d-vector 0 0 (- diff-z)))
             (tf:orientation pose-in-map))
            diff-z)))
        (t (values pose-stamped 0.0))))

(defun last-supporting-object (designator)
  (when designator
    (let* ((at (desig-prop-value designator 'desig-props:at))
           (supporting-object
             (when at (first
                       (sem-map-utils:designator->semantic-map-objects
                        at)))))
      (cond (supporting-object supporting-object)
            (t (last-supporting-object (parent designator)))))))

(defun pose-pointing-away-from-base (object-pose)
  (let ((ref-frame "/base_link")
        (fin-frame "/map"))
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
         (cons 'desig-props:cylinder (vector 0.08 0.14)))
        (t (cons 'desig-props:box (vector 0.06 0.09 0.2)))))

(defun get-object-name (parent pose type &key (threshold 0.2))
  (let ((near-objects
          (crs::force-ll
           (crs::lazy-mapcar
            (lambda (n)
              (crs::var-value '?n n))
            (crs:prolog `(and (btr::bullet-world ?w)
                              (btr::object ?w ?n)
                              (btr::object-pose ?w ?n ?p)
                              (btr::poses-equal ?p ,pose (,threshold 6.4))))))))
    (loop for i from 0 below (length near-objects)
          for temp-name = (indexed-name type i)
          when (find temp-name near-objects)
            do (return-from get-object-name temp-name))
    (format t "gogogo: ~a~%" near-objects)
    (let ((parent-name (desig-prop-value parent 'desig-props:name)))
      (cond (parent-name parent-name)
            (t (let ((all-objects
                       (crs::force-ll
                        (crs::lazy-mapcar
                         (lambda (n)
                           (crs::var-value '?n n))
                         (crs:prolog
                          `(and (btr::bullet-world ?w)
                                (btr::object ?w ?n)))))))
                 (loop for i from 0 to (length all-objects)
                       for temp-name = (indexed-name type i)
                       when (not (crs:prolog `(and (btr::bullet-world ?w)
                                                   (btr::object ?w ,temp-name))))
                         do (return-from get-object-name temp-name))))))))

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

(defun find-object (designator)
  (let* ((at (desig-prop-value designator 'desig-props:at))
         (pose-stamped (when at (desig-prop-value at 'desig-props:pose)))
         (pose-stamped-expected
           (when pose-stamped (moveit:ensure-pose-stamped-transformed
                               pose-stamped "/map" :ros-time t)))
         (supporting-object (last-supporting-object designator))
         (supporting-z-value
           (when supporting-object
             (semantic-map-costmap::obj-z-value supporting-object)))
         (objects (uima:get-uima-result designator)))
    (roslisp:ros-info (perception) "Processing ~a object(s)" (length objects))
    (loop for object in objects
          for type = (let ((type-sym (or (cram-designators:desig-prop-value
                                          object 'desig-props:type)
                                         'desig-props:pancakemix)))
                       (when type-sym
                         (intern (replace-all (string-upcase type-sym) "_" "")
                                 'desig-props)))
          for dimensions = (cdr (deduce-model-properties
                                 (cram-designators:desig-prop-value
                                  designator 'desig-props:dimensions)
                                 type))
          for pose-on-plane-pre = (when (cram-designators:desig-prop-value
                                         object 'desig-props:pose-on-plane)
                                    (let ((pose-value (tf:msg->pose-stamped
                                                       (cram-designators:desig-prop-value
                                                        object 'desig-props:pose-on-plane))))
                                      pose-value))
          for pose-on-plane-correction = (when pose-on-plane-pre
                                           (multiple-value-bind
                                                 (pose diff-z)
                                               (elevate-if-necessary
                                                pose-on-plane-pre
                                                supporting-z-value)
                                             (cons pose diff-z)))
          for pose-on-plane = (car pose-on-plane-correction)
          for needs-reorientation = (when (cram-designators:desig-prop-value object 'desig-props:pose)
                                      (let* ((pose-temp (tf:msg->pose-stamped
                                                         (cram-designators:desig-prop-value
                                                          object 'desig-props:pose)))
                                             (orientation (tf:orientation pose-temp)))
                                        (and (eql (tf:x orientation) 0.0d0)
                                             (eql (tf:y orientation) 0.0d0)
                                             (eql (tf:z orientation) 0.0d0)
                                             (eql (tf:w orientation) 1.0d0))))
          for pose = (when (cram-designators:desig-prop-value object 'desig-props:pose)
                       (let* ((pose-in-map (moveit:ensure-pose-stamped-transformed
                                            (tf:msg->pose-stamped
                                             (cram-designators:desig-prop-value
                                              object 'desig-props:pose))
                                            "/map" :ros-time t))
                              (pose-in-map-corr
                                (cond (needs-reorientation (pose-pointing-away-from-base pose-in-map))
                                      (t pose-in-map))))
                         (tf:make-pose-stamped
                          "/map" (tf:stamp pose-in-map-corr)
                          (tf:v+ (tf:origin pose-in-map-corr)
                                 (tf:make-3d-vector
                                  0 0 (cond (pose-on-plane
                                             (- (cdr pose-on-plane-correction)))
                                            (dimensions (/ (elt dimensions
                                                                (1- (length dimensions))) 1))
                                            (t 0.0))))
                          (tf:orientation pose-in-map-corr))))
          for name = (get-object-name designator pose type)
          for grasp-poses = (let ((gp (cram-designators:desig-prop-values
                                       object 'desig-props:grasp-pose)))
                              (mapcar
                               (lambda (g)
                                 (moveit:ensure-pose-stamped-transformed
                                  (tf:msg->pose-stamped g)
                                  "/map" :ros-time t))
                               gp))
          for z-offset = (cond ((and pose pose-on-plane-pre)
                                (- (tf:z (tf:origin pose))
                                   (tf:z (tf:origin pose-on-plane))))
                               ((and pose supporting-z-value)
                                (- (tf:z (tf:origin pose)) supporting-z-value))
                               (t 0.0))
          for shape = (car (deduce-model-properties
                            (cram-designators:desig-prop-value
                             designator 'desig-props:dimensions)
                            type))
          for color = (symbol-or-unknown object designator 'desig-props:color 'desig-props:yellow)
          for size = (symbol-or-unknown object designator 'desig-props:size)
          when (and pose
                    (or (not pose-stamped-expected)
                        (<= (tf:v-dist
                             (tf:make-3d-vector
                              (tf:x (tf:origin pose-stamped-expected))
                              (tf:y (tf:origin pose-stamped-expected)) 0)
                             (tf:make-3d-vector
                              (tf:x (tf:origin pose))
                              (tf:y (tf:origin pose)) 0))
                            *proximity-threshold*)))
            collect (progn
                      (register-object name pose)
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
                       :grasp-poses grasp-poses)))))

(defun found-object-fits-prototype (found-object prototype-object)
  (let ((prot-type (desig-prop-value prototype-object 'desig-props:type))
        (obj-type (desig-prop-value found-object 'desig-props:type)))
    (or (not prot-type)
        (eql prot-type obj-type))))

(defun find-with-designator (designator)
  (let ((found-objects
          (mapcar (lambda (perceived-object)
                    (perceived-object->designator perceived-object
                                                  :parent designator))
                  (find-object designator))))
    (loop for object in found-objects
          when (found-object-fits-prototype object designator)
            collect object)))

(defun register-object (name pose)
  (crs:prolog `(and (btr:bullet-world ?w)
                    (btr:assert
                     (btr:object
                      ?w btr:box
                      ,name
                      ((,(tf:x (tf:origin pose))
                         ,(tf:y (tf:origin pose))
                         ,(tf:z (tf:origin pose)))
                       (,(tf:x (tf:orientation pose))
                         ,(tf:y (tf:orientation pose))
                         ,(tf:z (tf:orientation pose))
                         ,(tf:w (tf:orientation pose))))
                      :size (0.1 0.1 0.1)
                      :mass 0.0))
                    ;; This gets asserted again because the
                    ;; assert object predicate doesn't update
                    ;; the object pose when the named object
                    ;; already exists. Bummer.
                    (btr:assert
                     ?w ,name
                     ((,(tf:x (tf:origin pose))
                       ,(tf:y (tf:origin pose))
                       ,(tf:z (tf:origin pose)))
                      (,(tf:x (tf:orientation pose))
                       ,(tf:y (tf:orientation pose))
                       ,(tf:z (tf:orientation pose))
                       ,(tf:w (tf:orientation pose))))))))

(def-action-handler perceive (object-designator)
  (let* ((perceived-designators
           (find-with-designator (or (newest-effective-designator
                                      object-designator)
                                     object-designator))))
    (unless perceived-designators
      (cpl:fail 'cram-plan-failures:object-not-found
                :object-desig object-designator))
    (dolist (designator perceived-designators)
      (moveit:register-collision-object
       designator :add t)
      ;(let ((pose (reference (desig-prop-value designator 'desig-props:at)))
       ;     (name (desig-prop-value designator 'desig-props:name)))
        ;(register-object name pose))
      (cram-plan-knowledge:on-event
       (make-instance
        'cram-plan-knowledge:object-perceived-event
        :perception-source :robosherlock
        :object-designator designator)))
    perceived-designators))

(def-process-module robosherlock-process-module (desig)
  (apply #'call-action (reference desig)))

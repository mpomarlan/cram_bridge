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

(defparameter *proximity-threshold* 0.4)
(defparameter *object-marker-color* `(1.0 0.0 0.0 1.0))

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

(defun elevate-if-necessary (pose-stamped supporting-z dimensions &key head-to-map)
  (declare (ignorable supporting-z dimensions))
  (let ((pose-in-map (cond (head-to-map
                             (tf:pose->pose-stamped
                              "/map" 0.0
                              (cl-transforms:transform-pose head-to-map pose-stamped)))
                           (t (moveit:ensure-pose-stamped-transformed
                               pose-stamped "/map" :ros-time t)))))
    (values pose-in-map 0.0)))
    ;; (cond (supporting-z
    ;;        (values pose-in-map supporting-z)
    ;; (cond ((and supporting-z (> 0.0 supporting-z))
    ;;        (let* ((diff-z (- (tf:z (tf:origin pose-in-map)) supporting-z))
    ;;               (pose-elevated (tf:copy-pose-stamped pose-in-map
    ;;                                                    :origin (tf:v- (tf:origin pose-stamped)
    ;;                                                                   (tf:make-3d-vector
    ;;                                                                    0.0 0.0 diff-z)))))
    ;;          (values pose-elevated diff-z)))
    ;;       (t (values pose-in-map 0.0)))))
    
  ;; (multiple-value-bind (pose-elevated diff-z)
  ;;     (cond (supporting-z
  ;;            (let* ((pose-in-map
  ;;                     (cond (head-to-map
  ;;                            (tf:pose->pose-stamped
  ;;                             "/map" 0.0
  ;;                             (cl-transforms:transform-pose head-to-map pose-stamped)))
  ;;                           (t (moveit:ensure-pose-stamped-transformed
  ;;                               pose-stamped "/map" :ros-time t))))
  ;;                   (pose-z (tf:z (tf:origin pose-in-map)))
  ;;                   (diff-z (- pose-z supporting-z)))
  ;;              (format t "popp: ~a, diff-z: ~a~%" pose-in-map diff-z)
  ;;              (values
  ;;               (tf:make-pose-stamped
  ;;                "/map" (tf:stamp pose-in-map)
  ;;                (tf:v+ (tf:origin pose-in-map)
  ;;                       (tf:make-3d-vector
  ;;                        0 0 (- diff-z)))
  ;;                (tf:orientation pose-in-map))
  ;;               diff-z)))
  ;;           (t (values pose-stamped 0.0)))
  ;;   (format t "dimensions: ~a, pose-elevated: ~a~%" dimensions pose-elevated)
  ;;   (values (tf:copy-pose-stamped
  ;;            pose-elevated
  ;;            :origin (tf:v+ (tf:origin pose-elevated)
  ;;                           (tf:make-3d-vector
  ;;                            0 0 0))) ;(/ (aref dimensions (1- (length dimensions))) 2))))
  ;;           (+ diff-z))))

(defun last-supporting-object (designator)
  (when designator
    (let* ((at (desig-prop-value designator 'desig-props:at))
           (supporting-object
             (when at (first
                       (sem-map-utils:designator->semantic-map-objects
                        at)))))
      ;(format t "SUPPORTING OBJECT: ~a~%" (slot-value supporting-object 'desig-props::name))
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
         (cons 'desig-props:cylinder (vector 0.07 0.15)))
        ((eql type 'desig-props:pancake)
         (cons 'desig-props:cylinder (vector 0.02 0.06)))
        (t (cons 'desig-props:box (vector 0.06 0.09 0.2)))))

(defun get-object-name (parent pose type &key (threshold 0.2))
  (format t "1: ~a~%" type)
  (let ((near-objects
          (crs::force-ll
           (crs::lazy-mapcar
            (lambda (n)
              (crs::var-value '?n n))
            (crs:prolog `(and (btr::bullet-world ?w)
                              (btr::object ?w ?n)
                              (not (equal ?n nil))
                              (btr::object-pose ?w ?n ?p)
                              (btr::poses-equal ?p ,pose (,threshold 6.4)))))))
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
    (format t "2~%")
    (let ((very-close-objects
            (crs::force-ll
             (crs::lazy-mapcar
              (lambda (n)
                (crs::var-value '?n n))
              (crs:prolog `(and (btr::bullet-world ?w)
                                (btr::object ?w ?n)
                                (btr::object-pose ?w ?n ?p)
                                (btr::poses-equal ?p ,pose (0.01 6.4)))))))
          (parent-name (desig-prop-value parent 'desig-props:name)))
      (format t "vco: ~a~%" very-close-objects)
      (let ((type (or (and
                       very-close-objects
                       (when (eql type 'desig-props:pancakemaker)
                         (dolist (close-object very-close-objects)
                           (crs:prolog `(and (btr:bullet-world ?w)
                                             (btr:retract ?w (btr:object ,close-object)))))
                         'desig-props:pancakemaker))
                      (and (not very-close-objects)
                           type))))
        (format t "Through~%")
        (when type
          (cond (parent-name parent-name)
                (t (loop for i from 0 to (length all-objects)
                         for temp-name = (indexed-name type i)
                         when (not (crs:prolog `(and (btr::bullet-world ?w)
                                                     (btr::object ?w ,temp-name))))
                           do (return-from get-object-name temp-name)))))))))

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
  ;; Wait for two seconds as UIMA needs a moment to produce concise,
  ;; current results.
  (cpl:sleep* 2)
  (let* ((at (desig-prop-value designator 'desig-props:at))
         (pose-stamped (when at (desig-prop-value at 'desig-props:pose)))
         (pose-stamped-expected
           (when pose-stamped (moveit:ensure-pose-stamped-transformed
                               pose-stamped "/map" :ros-time t)))
         (supporting-object (last-supporting-object designator))
         (supporting-z-value
           (when supporting-object
             (semantic-map-costmap::obj-z-value supporting-object)))
         (objects (uima:get-uima-result designator))
         (head-to-map-trafo (moveit:ensure-transform-available
                             "/head_mount_kinect_rgb_optical_frame" "/map")))
    (format t "Head to map trafo: ~a~%" head-to-map-trafo)
    (flet ((pose-head-to-map (pose-head)
             (tf:pose->pose-stamped
              "/map" 0.0
              (cl-transforms:transform-pose head-to-map-trafo pose-head))))
      (roslisp:ros-info (perception) "Processing ~a object(s)" (length objects))
      (let* ((registered-objects nil)
             (found-objects
               (loop for object in objects
                     for sssss = (format t "Object: ~a~%" object)
                     for type = (let ((type-sym (or (cram-designators:desig-prop-value
                                                     object 'desig-props:type)
                                                    'desig-props:pancakemix)))
                                  (when type-sym
                                    (intern (replace-all (string-upcase type-sym) "_" "")
                                            'desig-props)))
                     for dbg0 = (format t "Found object of type: ~a~%" type)
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
                     for pose-center = (when (cram-designators:desig-prop-value
                                              object 'desig-props:pose-center)
                                         (let ((pose-center (tf:msg->pose-stamped
                                                             (cram-designators:desig-prop-value
                                                              object 'desig-props:pose-center))))
                                           pose-center))
                     for pose-on-plane-correction = (when pose-on-plane-pre
                                                      (multiple-value-bind
                                                            (pose diff-z)
                                                          (elevate-if-necessary
                                                           pose-on-plane-pre
                                                           supporting-z-value
                                                           dimensions)
                                                        (cons pose diff-z)))
                     for pose-on-plane = (car pose-on-plane-correction)
                     for needs-reorientation = (when (cram-designators:desig-prop-value
                                                      object 'desig-props:pose)
                                                 (let* ((pose-temp (tf:msg->pose-stamped
                                                                    (cram-designators:desig-prop-value
                                                                     object 'desig-props:pose)))
                                                        (orientation (tf:orientation pose-temp)))
                                                   (or (eql type 'desig-props:pancake)
                                                       (and (eql (tf:x orientation) 0.0d0)
                                                            (eql (tf:y orientation) 0.0d0)
                                                            (eql (tf:z orientation) 0.0d0)
                                                            (eql (tf:w orientation) 1.0d0)))))
                     for pose = (when (cram-designators:desig-prop-value object 'desig-props:pose)
                                  (let* ((pose-in-map
                                           (cond ((and pose-center
                                                       (eql type 'desig-props:pancakemaker))
                                                  (tf:copy-pose-stamped
                                                   pose-center
                                                   :origin (tf:v+ (tf:origin pose-center)
                                                                  (tf:make-3d-vector 0 0 -0.035))))
                                                 (t (pose-head-to-map
                                                     (tf:msg->pose-stamped
                                                      (cram-designators:desig-prop-value
                                                       object 'desig-props:pose))))))
                                         (pose-in-map-corr
                                           (cond (needs-reorientation
                                                  (pose-pointing-away-from-base pose-in-map))
                                                 (t pose-in-map))))
                                    (tf:make-pose-stamped
                                     "/map" (tf:stamp pose-in-map-corr)
                                     (tf:v+ (tf:origin pose-in-map-corr)
                                            (tf:make-3d-vector
                                             (+
                                              (cond ((eql type 'desig-props:pancakemaker)
                                                     -0.0)
                                                    (t 0.0)))
                                             0
                                             (+
                                              (or (and pose-on-plane-correction
                                                       (cond ((> (tf:z
                                                                  (tf:origin
                                                                   (car pose-on-plane-correction)))
                                                                 (tf:z (tf:origin
                                                                        pose-in-map)))
                                                              (- (tf:z (tf:origin
                                                                        (car
                                                                         pose-on-plane-correction)))
                                                                 (tf:z (tf:origin
                                                                        pose-in-map))))
                                                             (t 0.0)))
                                                  0.0)
                                              (+ (cond ((eql type 'desig-props:pancakemaker)
                                                        0.02)
                                                       ((eql type 'desig-props:pancake)
                                                        0.02)
                                                       (t 0.0))))))
                                     (tf:orientation pose-in-map-corr))))
                     for name = (get-object-name designator pose type)
                     for grasp-poses = (let ((gp (cram-designators:desig-prop-values
                                                  object 'desig-props:grasp-pose)))
                                         (mapcar
                                          (lambda (g)
                                            (pose-head-to-map (tf:msg->pose-stamped g)))
                                        ;(moveit:ensure-pose-stamped-transformed
                                        ; (tf:msg->pose-stamped g)
                                        ; "/map" :ros-time t))
                                          gp))
                     for z-offset = (max (cond ((and pose pose-on-plane-pre)
                                                (- (tf:z (tf:origin pose))
                                                   (tf:z (tf:origin pose-on-plane))))
                                               ((and pose supporting-z-value)
                                                (- (tf:z (tf:origin pose)) supporting-z-value))
                                               (t 0.0))
                                         0.0)
                     for shape = (car (deduce-model-properties
                                       (cram-designators:desig-prop-value
                                        designator 'desig-props:dimensions)
                                       type))
                     for color = (symbol-or-unknown object designator
                                                    'desig-props:color 'desig-props:yellow)
                     for size = (symbol-or-unknown object designator 'desig-props:size)
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
                       collect (progn
                                 (push name registered-objects)
                                 (register-object name type pose dimensions)
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
        (dolist (registered-object registered-objects)
          (crs:prolog `(and (btr:bullet-world ?w)
                            (btr:retract ?w (btr:object ,registered-object)))))
        found-objects))))

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

(defun register-object (name type pose dimensions)
  (let ((pose (cl-transforms:transform-pose
               (tf:make-transform
                (tf:make-3d-vector
                 0.0 0.0 0.0)
                (tf:make-identity-rotation))
               pose)))
    (cond ((or (eql type 'desig-props:pancake)
               (eql type 'desig-props:pancakemaker))
           (crs:prolog `(and (btr:bullet-world ?w)
                             (btr:assert
                              (btr:object
                               ?w ,(cond ((eql type 'desig-props:pancake)
                                          'desig-props:pancake)
                                         (t 'btr::pancake-maker))
                               ,name ,(tf:copy-pose
                                       pose :origin (tf:v+ (tf:origin pose)
                                                           (tf:make-3d-vector
                                                            0 0 (/ (aref dimensions 0) -2))))
                               :size ,(cond ((eql type 'desig-props:pancakemaker)
                                             `(0.15 0.15 0.035))
                                            (t `(0.05 0.05 0.01)))
                               :mass 0.1)))))
          (t (crs:prolog `(and (btr:bullet-world ?w)
                               (btr:assert
                                (btr:object
                                 ?w btr:mesh ,name ,pose
                                 :mesh ,(cond ((eql type 'desig-props:pancakemix)
                                               'btr:mondamin)
                                              (t type))
                                 :mass 0.1))))))))

(def-action-handler perceive (object-designator)
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
             (object-pose (desig-prop-value object-at 'desig-props:pose)))
        (moveit:register-collision-object
         designator
         :add t
         :pose-stamped (tf:copy-pose-stamped
                        object-pose :origin (tf:v- (tf:origin object-pose)
                                                   (tf:make-3d-vector
                                                    0 0 (/ (aref object-dimensions
                                                                 (1- (length object-dimensions)))
                                                           4)))))
        (register-object
         object-name object-type object-pose object-dimensions)
        (moveit:set-object-color object-name
                                 *object-marker-color*))
      (cram-plan-knowledge:on-event
       (make-instance
        'cram-plan-knowledge:object-perceived-event
        :perception-source :robosherlock
        :object-designator designator)))
    perceived-designators))

(def-process-module robosherlock-process-module (desig)
  (apply #'call-action (reference desig)))

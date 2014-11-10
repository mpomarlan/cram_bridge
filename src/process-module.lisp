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

(defparameter *proximity-threshold* 0.025)
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
   (colors :reader colors :initarg :colors)
   (z-offset :reader z-offset :initarg :z-offset)
   (grasp-poses :reader grasp-poses :initarg :grasp-poses)
   (desig-props::pass-through-properties :reader pass-through-properties :initarg :pass-through-properties)
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
    (let ((colors (bound-slot-value perceived-object 'desig-props::colors))
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
          (grasp-type (desig-prop-value parent 'desig-props:grasp-type))
          (passed-through (bound-slot-value perceived-object 'desig-props::pass-through-properties)))
      (make-effective-designator
       parent
       :new-properties
       (append
        (refine-description
         (append
          `(,@(when colors
              (mapcar (lambda (color)
                        `(desig-props:color ,color))
                      colors)))
          (desig:update-designator-properties
           `(,@(when shape `((desig-props:shape ,shape)))
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
               ,@passed-through
               (desig-props:at
                ,(desig:make-designator
                  'desig:location `((desig-props:pose ,pose))))
               ,@(when identifier `((desig-props:name ,identifier))))
           (when parent (desig:properties parent))))))
       :data-object perceived-object))))

(defun symbol-or-unknown (found-designator parent-designator symbol
                          &optional (default 'unknown) multiple)
  (cond (multiple
         (mapcar (lambda (annotation)
                   (cond ((eql symbol 'desig-props:color)
                          (cond ((listp annotation)
                                 `(,(intern (string-upcase
                                             (cadar annotation))
                                            :desig-props)
                                   ,(cadadr annotation)))
                                (t `(,(intern (string-upcase annotation)
                                              :desig-props) 1.0))))
                         (t nil)))
                 (desig-prop-values found-designator symbol)))
        (t
         (let ((prop-value (cram-designators:desig-prop-value
                            found-designator symbol)))
           (cond ((eql symbol 'desig-props:color)
                  (cond ((listp prop-value)
                         `(,(intern (string-upcase
                                     (cadar prop-value)))
                           ,(cadadr prop-value)))
                        (t `(,(intern (string-upcase prop-value)
                                      :desig-props) 1.0))))
                 (t (intern (string-upcase (or prop-value
                                               (desig-prop-value
                                                parent-designator
                                                symbol)
                                               default))
                            :desig-props)))))))

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

(cut:define-hook cram-language::on-begin-object-identity-resolution (type))
(cut:define-hook cram-language::on-finish-object-identity-resolution (log-id name))

(defun get-object-name (parent pose type &key (threshold *proximity-threshold*))
  (let* ((type 'desig-props:object)
         (log-id (first (cram-language::on-begin-object-identity-resolution type)))
         (pose-in-map
           (when pose (moveit:ensure-pose-stamped-transformed pose "/map")))
         (resolved-name
           (block name-resolution
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
                                           (btr::poses-equal
                                            ?p ,pose-in-map
                                            (,threshold 6.4))))))))
                   (all-objects
                     (crs::force-ll
                      (crs::lazy-mapcar
                       (lambda (n)
                         (crs::var-value '?n n))
                       (crs:prolog
                        `(and (btr::bullet-world ?w)
                              (btr::object ?w ?n)))))))
               (format t "Near objects: ~a~%Vs. All objects: ~a~%"
                       near-objects all-objects)
               (loop for i from 0 below (length all-objects)
                     for temp-name = (indexed-name type i)
                     when (find temp-name near-objects)
                       do (return-from name-resolution temp-name))
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
                                             (btr::poses-equal ?p ,pose-in-map
                                                               (0.01 6.4))))))))
                     (parent-name (desig-prop-value parent 'desig-props:name)))
                 (format t "VCO: ~a~%Parent: ~a~%"
                         very-close-objects parent-name)
                 (let ((type (or (and
                                  very-close-objects
                                  (when (eql type 'desig-props:pancakemaker)
                                    (dolist (close-object very-close-objects)
                                      (crs:prolog
                                       `(and (btr:bullet-world ?w)
                                             (btr:retract ?w (btr:object
                                                              ,close-object)))))
                                    'desig-props:pancakemaker))
                                 (and (not very-close-objects)
                                      type))))
                   (when type
                     (cond (parent-name parent-name)
                           (t (loop for i from 0 to (length all-objects)
                                    for temp-name = (indexed-name type i)
                                    when (not
                                          (crs:prolog
                                           `(and (btr::bullet-world ?w)
                                                 (btr::object ?w ,temp-name))))
                                      do (return-from name-resolution
                                           temp-name)))))))))))
    (cram-language::on-finish-object-identity-resolution log-id resolved-name)
    resolved-name))

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
  (let ((waiting-time 1.5))
    (when (> waiting-time 0)
      (roslisp:ros-info (robosherlock) "Waiting for perception to settle on correct images (~a sec)." waiting-time)
      (cpl:sleep waiting-time)
      (roslisp:ros-info (robosherlock) "Continuing.")))
  (let ((objects (uima:get-uima-result designator))
        (default-object-type 'desig-props:pancakemix)
        (registered-objects nil)
        (pass-through-properties `()))
    (prog1
        (mapcar (lambda (object)
                  (let* ((type (or (intern
                                    (replace-all (string-upcase
                                                  (cram-designators:desig-prop-value
                                                   object 'desig-props:type)) "_" "")
                                    'desig-props) default-object-type))
                         (dimensions
                           (let ((dim (desig-prop-value
                                       object 'desig-props::dimensions)))
                             (vector
                              (cadr (find
                                     'desig-props::height
                                     dim :test (lambda (x y)
                                                 (eql x (car y)))))
                              (cadr (find
                                     'desig-props::width
                                     dim :test (lambda (x y)
                                                 (eql x (car y)))))
                              (cadr (find
                                     'desig-props::depth
                                     dim :test (lambda (x y)
                                                 (eql x (car y))))))))
                         (needs-reorientation
                           (not (or (eql type 'spatula))))
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
                         (z-offset
                           (max (cond ((and pose pose-on-plane)
                                       (tf:v-dist (tf:origin pose)
                                                  (tf:origin pose-on-plane)))
                                      (t 0.0))
                                0.0))
                         (shape (or (desig-prop-value designator
                                                      'desig-props:shape)
                                    'desig-props:box))
                         (colors (symbol-or-unknown
                                  object designator
                                  'desig-props:color
                                  `(desig-props:yellow 1.0)
                                  t))
                         (size (symbol-or-unknown object designator
                                                  'desig-props:size))
                         (passed-through (loop for prop in (description object)
                                               when (destructuring-bind (key val)
                                                        prop
                                                      (declare (ignore val))
                                                      (find key pass-through-properties))
                                                 collect prop)))
                    (push name registered-objects)
                    (register-object name type pose dimensions
                                     :z-offset z-offset)
                    (make-instance
                     'perceived-object
                     :object-identifier name
                     :identifier name
                     :pose pose
                     :type type
                     :shape shape
                     :colors colors
                     :size size
                     :z-offset z-offset
                     :dimensions dimensions
                     :grasp-poses grasp-poses
                     :pass-through-properties passed-through)))
                objects)
      (dolist (registered-object registered-objects)
        (crs:prolog `(and (btr:bullet-world ?w)
                          (btr:retract ?w (btr:object ,registered-object))))))))

(defun find-object (designator)
  (let* ((perceived-objs (get-perceived-objects designator))
         (registered-objects nil))
    (let* ((at (desig-prop-value (desig:current-desig designator) 'desig-props:at))
           (pose-stamped (when at (desig-prop-value at 'desig-props:pose)))
           (pose-stamped-expected
             (when pose-stamped (moveit:ensure-pose-stamped-transformed
                                 pose-stamped *object-reference-frame* :ros-time t)))
           (valid-objects
             (loop for object in perceived-objs
                   for name = (slot-value object 'identifier)
                   for pose = (moveit:ensure-pose-stamped-transformed
                               (slot-value object 'pose) *object-reference-frame*)
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

(cut:define-hook cram-language::on-prepare-request (designator-request))
(cut:define-hook cram-language::on-finish-request (log-id designators-result))

(defun find-with-designator (designator)
  (let* ((log-id (first (cram-language::on-prepare-request designator)))
         (found-objects
           (mapcar (lambda (perceived-object)
                     (perceived-object->designator perceived-object
                                                   :parent designator))
                   (find-object designator)))
         (fitting-objects
           (loop for object in found-objects
                 when (found-object-fits-prototype object designator)
                   collect object)))
    (cram-language::on-finish-request log-id fitting-objects)
    fitting-objects))

(cut:define-hook cram-language::on-begin-belief-state-update ())
(cut:define-hook cram-language::on-finish-belief-state-update (id))

(defun register-object (name type pose dimensions &key (z-offset 0.0))
  (declare (ignorable dimensions))
  (let ((log-id (first (cram-language::on-begin-belief-state-update))))
    (unwind-protect
         (let ((pose (moveit:ensure-pose-stamped-transformed pose "/map")))
           (cond ((or (eql type 'desig-props:pancake)
                      (eql type 'desig-props:pancakemaker))
                  (crs:prolog
                   `(and (btr:bullet-world ?w)
                         (btr:assert
                          (btr:object
                           ?w ,(cond ((eql type 'desig-props:pancake)
                                      'desig-props:pancake)
                                     (t 'btr::pancake-maker))
                           ,name ,(tf:copy-pose
                                   pose
                                   :origin
                                   (cond ((eql type 'desig-props:pancakemaker)
                                          (tf:v- (tf:origin pose)
                                                 (tf:make-3d-vector
                                                  0 0 z-offset)))
                                         (t (tf:v- (tf:origin pose)
                                                   (tf:make-3d-vector
                                                    0 0 0.035)))))
                           :size ,(cond ((eql type 'desig-props:pancakemaker)
                                         `(0.15 0.15 0.035))
                                        (t `(0.05 0.05 0.01)))
                           :mass 0.1)))))
                 (t (format t "~a~%" dimensions)
                    (crs:prolog `(and (btr:bullet-world ?w)
                                      (btr:assert
                                       (btr:object
                                        ?w btr:box
                                        ,name ,(tf:copy-pose
                                                pose
                                                :origin
                                                (tf:v- (tf:origin pose)
                                                       (tf:make-3d-vector
                                                        0 0 (/ z-offset 2))))
                                        :mass 0.0
                                        :size ,(map 'list #'identity dimensions))))))))
      
                                        ;; ?w btr:mesh
                                        ;; ,name ,(tf:copy-pose
                                        ;;         pose
                                        ;;         :origin
                                        ;;         (tf:v- (tf:origin pose)
                                        ;;                (tf:make-3d-vector
                                        ;;                 0 0 (/ z-offset 2))))
                                        ;; :mesh ,(cond ((eql
                                        ;;                type
                                        ;;                'desig-props:pancakemix)
                                        ;;               'btr:mondamin)
                                        ;;              (t 'btr:mondamin));type))
                                        ;; :mass 0.1)
      (cram-language::on-finish-belief-state-update log-id))))

(defun pose-on-surface (pose-obj z-offset &key
                                            (allowed-types nil allowed-types-p)
                                            original-dimensions
                                            type)
  (let* ((sem-objs
           (cut:var-value
            '?o
            (first
             (crs:prolog
              `(and (btr:bullet-world ?w)
                    (semantic-map-costmap:semantic-map-objects ?o))))))
         (surfaces
           (loop for sem-obj in sem-objs
                 for dimensions = (slot-value
                                   sem-obj 'semantic-map-utils:dimensions)
                 for pose-sem = (moveit:ensure-pose-stamped-transformed
                                 (tf:pose->pose-stamped
                                  "/map" 0.0
                                  (slot-value sem-obj 'semantic-map-utils:pose))
                                 "/odom_combined")
                 for origin-sem = (tf:origin pose-sem)
                 for origin-obj = (tf:origin pose-obj)
                 for half-width = (/ (tf:x dimensions) 2)
                 for half-depth = (/ (tf:y dimensions) 2)
                 when (and (<= (tf:z origin-sem) (tf:z origin-obj))
                           (<= (tf:x origin-obj) (+ (tf:x origin-sem) half-width))
                           (>= (tf:x origin-obj) (- (tf:x origin-sem) half-width))
                           (<= (tf:y origin-obj) (+ (tf:y origin-sem) half-depth))
                           (>= (tf:y origin-obj) (- (tf:y origin-sem) half-depth))
                           (or (not allowed-types-p)
                               (find (slot-value sem-obj 'common-lisp:type)
                                     allowed-types :test #'string=)))
                   collect sem-obj))
         (sorted-surfaces
           (sort surfaces (lambda (s1 s2)
                            (> (tf:z (tf:origin (slot-value s1 'semantic-map-utils:pose)))
                               (tf:z (tf:origin (slot-value s2 'semantic-map-utils:pose))))))))
    (let ((real-pose
            (cond (sorted-surfaces
                   (tf:copy-pose-stamped
                    pose-obj :origin (tf:make-3d-vector
                                      (tf:x (tf:origin pose-obj))
                                      (tf:y (tf:origin pose-obj))
                                      (+ (tf:z (tf:origin (slot-value (first sorted-surfaces)
                                                                      'semantic-map-utils:pose)))
                                         z-offset
                                         (/ (tf:z (slot-value (first sorted-surfaces)
                                                              'semantic-map-utils:dimensions))
                                            2)))))
                  (t pose-obj))))
      (values
       real-pose
       (cond ((or (eql type 'pancakemaker)
                  (eql type 'spatula))
              original-dimensions)
             (t original-dimensions))))))

(def-action-handler perceive (object-designator)
  (ros-info (perception) "Perceiving object.")
  (let ((results (perceive-object-designator object-designator)))
    (unless results
      (cpl:fail 'cram-plan-failures:object-not-found
                :object-desig object-designator))
    results))

(def-action-handler examine (object-designator)
  (declare (ignorable object-designator))
  (ros-info (perception) "Examining object."))

(def-process-module robosherlock-process-module (desig)
  (apply #'call-action (reference desig)))

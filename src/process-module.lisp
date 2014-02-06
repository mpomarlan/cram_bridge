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
   (dimensions :reader dimensions :initarg :dimensions)))

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
          (type (bound-slot-value perceived-object 'desig-props:type))
          (z-offset (bound-slot-value perceived-object 'desig-props:z-offset))
          (dimensions (bound-slot-value perceived-object
                                        'desig-props:dimensions))
          (grasp-poses (bound-slot-value perceived-object
                                         'desig-props:grasp-poses)))
      (make-effective-designator
       parent
       :new-properties (desig:update-designator-properties
                        `(,@(when color `((desig-props:color ,color)))
                          ,@(when shape `((desig-props:shape ,shape)))
                          ,@(when size `((desig-props:size ,size)))
                          ,@(when type `((desig-props:type ,type)))
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

(defun find-object (designator)
  (let* ((at (desig-prop-value designator 'desig-props:at))
         (pose-stamped (when at (desig-prop-value at 'desig-props:pose)))
         (pose-stamped-expected
           (when pose-stamped (moveit:ensure-pose-stamped-transformed
                               pose-stamped "/map" :ros-time t))))
    (let ((objects (uima:get-uima-result designator)))
      (roslisp:ros-info (perception) "Processing objects")
      (loop for object in objects
            for dimensions = (or (cram-designators:desig-prop-value
                                  designator 'desig-props:dimensions)
                                 (vector 0.1 0.1 0.1))
            for name = (symbol-or-unknown object designator 'desig-props:name
                                          'object)
            for type = (let ((type-sym (cram-designators:desig-prop-value
                                        object 'desig-props:type)))
                         (when (intern
                                (string-upcase type-sym))))
            for pose = (moveit:ensure-pose-stamped-transformed
                        (tf:msg->pose-stamped
                         (cram-designators:desig-prop-value
                          object 'desig-props:pose))
                        "/map" :ros-time t)
            for pose-on-plane = (moveit:ensure-pose-stamped-transformed
                                 (tf:msg->pose-stamped
                                  (cram-designators:desig-prop-value
                                   object 'desig-props:pose-on-plane))
                                 "/map" :ros-time t)
            for grasp-poses = (let ((gp (cram-designators:desig-prop-values
                                         object 'desig-props:grasp-pose)))
                                (mapcar
                                 (lambda (g)
                                   (moveit:ensure-pose-stamped-transformed
                                    (tf:msg->pose-stamped g)
                                    "/map" :ros-time t))
                                 gp))
            for z-offset = (- (tf:z (tf:origin pose))
                              (tf:z (tf:origin pose-on-plane)))
            for shape = (symbol-or-unknown object designator 'desig-props:shape)
            for color = (symbol-or-unknown object designator 'desig-props:color)
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
              collect (make-instance
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

(defun find-with-designator (designator)
  (mapcar (lambda (perceived-object)
            (perceived-object->designator perceived-object
                                          :parent designator))
          (find-object designator)))

(def-action-handler perceive (object-designator)
  (or
   (let* ((perceived-designators
            (find-with-designator (or (newest-effective-designator
                                       object-designator)
                                      object-designator))))
     (dolist (designator perceived-designators)
       (moveit:register-collision-object designator :add t)
       (let ((pose (reference (desig-prop-value designator 'desig-props:at)))
             (name (desig-prop-value designator 'desig-props:name)))
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
                             :mass 0.0)))))
       (cram-plan-knowledge:on-event
        (make-instance
         'cram-plan-knowledge:object-perceived-event
         :perception-source :robosherlock
         :object-designator designator)))
     perceived-designators)
   (cpl:fail 'cram-plan-failures:object-not-found
             :object-desig object-designator)))

(def-process-module robosherlock-process-module (desig)
  (apply #'call-action (reference desig)))

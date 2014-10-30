;;; Copyright (c) 2014, Georg Bartels <georg.bartels@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
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

(in-package :beasty-command-interface)

(defparameter *collision-types* 
  (list :NO-COLLISION :CONTACT :LIGHT-COLLISION :STRONG-COLLISION :SEVERE-COLLISION)
  "List of symbols denoting all types of collisions beasty can report back.")

(defparameter *reaction-types*
  (list :IGNORE :ZERO-G :JOINT-IMP :SOFT-STOP :HARD-STOP)
  "List of symbols denoting all safety reactions beasty can exhibit.")

(defparameter *collision-index-map*
  (let ((collisions (make-hash-table)))
    (setf (gethash :CONTACT collisions) 0)
    (setf (gethash :LIGHT-COLLISION collisions) 1)
    (setf (gethash :STRONG-COLLISION collisions) 2)
    (setf (gethash :SEVERE-COLLISION collisions) 3)
    collisions)
  "Map to look up index of collision types (symbols) in beasty vector.")

(defparameter *collision-symbol-map*
  (let ((map (make-hash-table)))
    (loop for k being the hash-key in *collision-index-map* using (hash-value v) do
      (setf (gethash v map) k))
    map)
  "Map to look up collision types (symbols) given their index in a beasty vector.")

(defparameter *reaction-code-map*
  (let ((map (make-hash-table)))
    (setf (gethash :IGNORE map) 0)
    (setf (gethash :SOFT-STOP map) 1)
    (setf (gethash :HARD-STOP map) 2)
    (setf (gethash :JOINT-IMP map) 3)
    (setf (gethash :ZERO-G map) 4)
    map)
  "Map to lookup the beasty code for reaction strategies (symbols).")

(defparameter *default-thresholds*
  #(0.05 0.1 0.15 0.2 0.0 0.0 0.0 0.0)
  "Default collision detection thresholds for (in that order) :CONTACT, :LIGHT-COLLISION,
 :STRONG-COLLISION, :SEVERE-COLLISION, and 4x off. Values are in percent of maximum torque
 per joint.")

(defun collision-type-valid-p (collision-type)
  "Checks whether the symbol `collision-type' denotes an exisiting type of collision."
  (declare (type symbol collision-type))
  (member collision-type *collision-types*))

(defun reaction-type-valid-p (reaction-type)
  "Checks whether the symbol `reaction-type' denotes an existing type of reaction."
  (declare (type symbol reaction-type))
  (member reaction-type *reaction-types*))

(defun reflex-content-valid-p (collision-type reaction-type)
  "Checks whether the symbols `collision-type' and `reaction-type' form a valid collision
 strategy specification, i.e. beasty reflex."
  (declare (type symbol collision-type reaction-type))
  (and (collision-type-valid-p collision-type)
       (reaction-type-valid-p reaction-type)))

(defun make-beasty-reflex (collision reaction)
  "Creates an instance of class 'beasty-reflex' if symbol `collision' denotes a valid type
 of collision, and `reaction' denotes a valid type of reaction."
  (declare (type symbol collision reaction))
  (when (reflex-content-valid-p collision reaction)
    (make-instance 'beasty-reflex :collision-type collision :reaction-type reaction)))

(defun store-reflex (settings reflex)
  "Associates 'collision-type' of `reflex' with `reflex' in beasty parameter `settings'
 which is of type 'safety-settings'."
  (declare (type safety-settings settings)
           (type beasty-reflex reflex))
  (setf (gethash (collision-type reflex) (reflexes settings)) reflex))

(defun make-safety-settings (&optional reflexes)
  "Creates an instance of type 'safety-settings'. `reflexes' is an optional list of objects
 of type 'beasty-reflexes' which indicate how the control shall react to collisions."
  (declare (type list reflexes))
  (let ((settings (make-instance 'safety-settings)))
    (loop for reflex in reflexes do
      (store-reflex settings reflex))
    settings))

(defparameter *default-safety-settings*
  (make-safety-settings
   (list
    (make-beasty-reflex :CONTACT :IGNORE)
    (make-beasty-reflex :LIGHT-COLLISION :JOINT-IMP)
    (make-beasty-reflex :STRONG-COLLISION :SOFT-STOP)
    (make-beasty-reflex :SEVERE-COLLISION :HARD-STOP)))
  "Default safety settings for beasty controller.")

(defun has-collision-type-p (settings collision-type)
  "Checks whether `settings' holds a specification on how to react to `collision-type'."
  (declare (type safety-settings settings)
           (type symbol collision-type))
  (multiple-value-bind (value present) (gethash collision-type (reflexes settings))
    (declare (ignore value))
    present))

(defun remove-reflex (settings collision-type)
  "Removes a specified reflex associated with `collision-type' from `settings'."
  (declare (type safety-settings settings)
           (type symbol collision-type))
  (when (has-collision-type-p settings collision-type)
    (remhash collision-type (reflexes settings))))

(defun reflex-valid-p (reflex)
  "Checks whether `reflex' of type 'beasty-reflex' is valid."
  (declare (type beasty-reflex reflex))
  (reflex-content-valid-p (collision-type reflex) (reaction-type reflex)))

(defun all-reflexes-valid-p (settings)
  "Checks whether all reflexes stored in `settings' are valid."
  (declare (type safety-settings settings))
  (every 
   #'identity
   (loop for collision being the hash-key in (reflexes settings) using (hash-value reflex)
         collect (and (reflex-valid-p reflex) (eql collision (collision-type reflex))))))

(defun safety-settings-valid-p (settings)
  "Checks whether the beasty 'safety-settings' `settings' are valid, i.e. whether all
 reflexes in slot 'reflexes' are valid."
  (declare (type safety-settings settings))
  (and (> 9 (hash-table-count (reflexes settings)))
       (all-reflexes-valid-p settings)))
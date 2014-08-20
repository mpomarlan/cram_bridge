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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
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

(in-package :cl-user)

(desig-props:def-desig-package
 cram-beliefstate
 (:documentation "CRAM Belief State Interface")
 (:nicknames :beliefstate)
 (:use
  #:common-lisp
  #:cram-roslisp-common
  #:cram-reasoning
  #:cram-process-modules
  #:crs
  #:cut
  #:desig
  #:designators-ros
  #:roslisp
  #:cram-plan-failures)
  (:export
   ;; Functions
   beliefstate-init
   start-node
   stop-node
   extract-dot-file
   extract-owl-file
   extract-files
   set-color-usage
   add-object-to-active-node
   add-topic-image-to-active-node
   add-failure-to-active-node
   add-designator-to-active-node
   equate-designators
   set-metadata
   ;; Interactive objects
   register-interactive-object
   unregister-interactive-object
   set-interactive-object-menu
   set-interactive-object-pose
   ;; Interactive callbacks
   register-interactive-callback
   unregister-interactive-callback
   ;; Experiment metadata
   begin-experiment
   end-experiment
   query-input
   ;; General
   enable-logging
   toggle-logging)
  (:designator-properties))

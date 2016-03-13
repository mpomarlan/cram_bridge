;;; Copyright (c) 2014, Jan Winkler <winkler@cs.uni-bremen.de>
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
;;;     * Neither the name of Universität Bremen, nor the names of its
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

(defsystem robosherlock-process-module
  :author "Jan Winkler <winkler@uni-bremen.de>"
  :license "BSD"
  :description "A package to connect CRAM to the RoboSherlock perception system."

  :depends-on (roslisp
               alexandria
               process-modules
               designators
               cram-plan-knowledge
               cram-projection
               cram-roslisp-common
               cram-language
               cram-manipulation-knowledge
               cram-plan-failures
               cl-semantic-map-utils
               semantic-map-costmap
               actionlib
               cram-uima
               cram-task-knowledge)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "facts" :depends-on ("package"))
     (:file "action-designator" :depends-on ("package" "facts"))
     (:file "interface" :depends-on ("package" "facts" "action-designator"))
     (:file "process-module"
      :depends-on ("package" "facts" "action-designator" "interface"))))))

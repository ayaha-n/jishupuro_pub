#!/usr/bin/env roseus
(defun utsumuki (&key (n 1)     ;; repeat count                                 
                 (tm 20)   ;; time(frame count)                                 
                 (v 30) ;; relative angle(deg)                                  
                 (j :head-neck-p) ;; joint-method                               
                 (mi 100))                                                      
  (send *robot* :copy-from-ri) ;; 実機から*robot*へ                             
  (dotimes (i n)                                                                
    (send *robot* j :joint-angle v) ;; 関節jの角度を与える                      
    (send *robot* :send-to-ri tm) ;; *robot*から実機へ                          
    ;;(send *robot* j :joint-angle (- v))                                       
    ;;(send *robot* :send-to-ri tm)                                             
    )                                                                           
  ;;wait
  (unix:sleep 2)
  (send *robot* j :joint-angle 0)                                               
  (send *robot* :send-to-ri tm)                                                 
  )                                                                             
                                                                                
(defun banzai (&key (n 2)                                                       
                    (tm 20)   ;; time(frame count)                              
                    (v -80) ;; relative angle(deg)                              
                    (j :rarm-shoulder-p) ;; joint-method                        
                    (k :larm-shoulder-p)                                        
                    (mi 100))                                                  
                                                                                
  (send *robot* :copy-from-ri) ;; 実機から*robot*へ                             
  (dotimes (i n)                                                                
    (send *robot* j :joint-angle v)                                             
    (send *robot* k :joint-angle v);; 関節jの角度を与える                       
    (send *robot* :send-to-ri tm) ;; *robot*から実機へ                          
    (send *robot* j :joint-angle 0)                                             
    (send *robot* k :joint-angle 0)                                             
    (send *robot* :send-to-ri tm)                                               
    )                                                                           
  )                                                                             
                                                                                
                                                                                
(defun kachikachi (&key (n 2)     ;; repeat count                               
                        (tm 20)   ;; time(frame count)                          
                        (v 20) ;; relative angle(deg)                           
                        (j :rarm-gripper2-r)                                    
                        (k :larm-gripper2-r);; joint-method                     
                        (mi 100))       
  (send *robot* :copy-from-ri) ;; 実機から*robot*へ                             
  (dotimes (i n)                                                                
    (send *robot* j :joint-angle v) ;; 関節jの角度を与える                      
    (send *robot* k :joint-angle v)                                             
    (send *robot* :send-to-ri tm) ;; *robot*から実機へ                          
    (send *robot* j :joint-angle (- v))                                         
    (send *robot* k :joint-angle (- v))                                         
    (send *robot* :send-to-ri tm)                                               
    )                                                                          
                                                                                
  (send *robot* j :joint-angle 0)
  (send *robot* k :joint-angle 0)
  (send *robot* :send-to-ri tm)                                                

)
                                                                                
(defun cb (msg)                                    
  (setq eye-status (send msg :data))                                                                  (print eye-status)
  (cond                                                                   
   ((= eye-status 5)                                                     
    (utsumuki)                                                           
    )                                                                    
   ((= eye-status 3)                                                     
    (utsumuki)                                                           
    )                                                                    
   ((= eye-status 4)                                                     
    (kachikachi)                                                         
    )                                                                    
   ((= eye-status 6)                                                     
    (banzai)                                                             
    )                                                                     
   ))                                                                 
                                                                                
                                                                                
                                                                                
(load "~/rcb4eus/rcb4robots.l")                                                 
(make-kxr-robot "kxrl2l2a6h2m")

(ros::load-ros-manifest "roseus")                                              
(ros::roseus "action_sub")

(print "(send *ri* :com-init)                                                           ")
(send *ri* :com-init)                                                           
;;(send *ri* :timer-on)
(print "(send *ri* :hold)")
(send *ri* :hold)                                                               
(send *ri* :neutral)
(ros::subscribe "/eye_status" std_msgs::UInt16 #'cb)

(do-until-key
 (ros::spin)
 )



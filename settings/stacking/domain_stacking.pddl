; Domain Description
; 
; This domain is for building towers using Pick and Place actions.
; Blocks may start anywhere on the table or in configurations that 
; have towers themselves.
;
; There are two types of objects in this world: Blocks and the Table.
; Blocks can be moved but the Table is fixed. The On(?o1 ?o2) predicate
; keeps tracks of which Blocks are on Tables or other objects.


(define (domain tool-use)
  (:requirements :strips :equality)
  (:predicates
    ; Types
    (Arm ?a)
    (Block ?b)
    (Table ?o)      
    (Pose ?o ?p)    
    (Grasp ?o ?g)
    
    ; Kin: Object ?o at Pose ?p can be grasped at Grasp ?g achievable with config ?q and traj ?t
    (Kin ?o ?p ?g ?q ?t)                
    ; FreeMotion: ?t is a traj between configurations ?q1 and ?q2
    (FreeMotion ?q1 ?t ?q2)
    ; HoldingMotion: ?t is a traj between ?q1 and ?q2 while holding object ?o with grasp ?g 
    (HoldingMotion ?q1 ?t ?q2 ?o ?g)
    ; Supported: Block ?o at Pose ?p1 will be supported by Table or Object ?o2 at Pose ?p2
    (Supported ?o1 ?p1 ?o2 ?p2)
    (ObjCFreeTraj ?a ?t ?o ?p)

    
    ; Fluents 
    (On ?o1 ?o2)
    (AtPose ?o ?p)
    (AtGrasp ?o ?g)
    (HandEmpty)
    (AtConf ?q)
    (CanMove)

    ; Derived
    ; Stackable: An object (Table or Block) is Stackable if an object can be placed on top 
    ;            of it. Blocks can only have one object placed on them. A Table is always Stackable
    ;            and a Block is Stackable if there is nothing on it.
    (Stackable ?o)
    ; Movable: Blocks that have nothing on top of them are Movable.
    (Movable ?o)
    ; UnSafeTraj: Trajectory ?t collides with some object.
    (UnsafeTraj ?t)
  )

  (:action move_free
    :parameters (?a ?q1 ?q2 ?t)
    :precondition (and (FreeMotion ?q1 ?t ?q2) ;(not (UnsafeTraj ?t))
                       (Arm ?a) (AtConf ?q1) (HandEmpty) (CanMove))
    :effect (and (AtConf ?q2)
                 (not (AtConf ?q1)) (not (CanMove)))
  )

  (:action move_holding
    :parameters (?a ?q1 ?q2 ?o ?g ?t)
    :precondition (and (HoldingMotion ?q1 ?t ?q2 ?o ?g) ;(not (UnsafeTraj ?t)))
                       (Arm ?a) (AtConf ?q1) (AtGrasp ?o ?g) (CanMove))
    :effect (and (AtConf ?q2)
                 (not (AtConf ?q1)) (not (CanMove)))
  )
    
  ; Pick up Block ?o1 at Pose ?p1 from Object (Block or Table) ?o2
  (:action pick
    :parameters (?a ?o1 ?p1 ?o2 ?g ?q ?t)
    :precondition (and (Kin ?o1 ?p1 ?g ?q ?t) 
                       (Arm ?a) (AtPose ?o1 ?p1) (HandEmpty) (AtConf ?q) 
                       (Movable ?o1) (not (CanMove)) (On ?o1 ?o2))
    :effect (and (AtGrasp ?o1 ?g) (CanMove)
                 (not (AtPose ?o1 ?p1)) (not (HandEmpty))
                 (not (On ?o1 ?o2)))
  )
  
  ; Place Block ?o at Pose ?p1 on Object (Block or Table) ?o2 which is at Pose ?p2
  (:action place
    :parameters (?a ?o1 ?p1 ?o2 ?p2 ?g ?q ?t)
    :precondition (and (Kin ?o1 ?p1 ?g ?q ?t)
                       (Arm ?a) (AtGrasp ?o1 ?g) (AtConf ?q) 
                       (Supported ?o1 ?p1 ?o2 ?p2) 
                       (AtPose ?o2 ?p2) (Stackable ?o2) (not (CanMove)))
    :effect (and (AtPose ?o1 ?p1) (HandEmpty) (CanMove) 
                 (not (AtGrasp ?o1 ?g)) (On ?o1 ?o2))
  )

  (:derived (UnsafeTraj ?t)
      (exists (?r ?o ?p ?g) (and (Arm ?r) (Pose ?o ?p) (AtPose ?o ?p)
                              (not (AtGrasp ?o ?g))
                              (not (ObjCFreeTraj ?r ?t ?o ?p))))
  )

  (:derived (Stackable ?o1)
    (or (forall (?o2) (not (On ?o2 ?o1))) (Table ?o1))
  )

  (:derived (Movable ?o)
    (and (Block ?o) (not (exists (?o2) (On ?o2 ?o))))
  )
)

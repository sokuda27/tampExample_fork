; Domain for pushing blocks along the table surface to a goal region.
; The robot keeps its hand closed and uses the end effector to push.

(define (domain tool-use)
  (:requirements :strips :equality)
  (:predicates
    ; Types
    (Arm ?a)
    (Block ?b)
    (Table ?o)
    (Region ?o)
    (Pose ?o ?p)

    ; PushMotion: certified by plan-push-motion stream
    (PushMotion ?o ?p1 ?t ?p2)
    ; Supported: block ?o at pose ?p is stably supported by surface ?s
    (Supported ?o ?p ?s)

    ; Fluents
    (On ?o1 ?o2)
    (AtPose ?o ?p)
    (HandEmpty)
    (CanMove)

    ; Derived
    (Movable ?o)
  )

  ; Push block ?o from pose ?p1 on surface ?s1 to pose ?p2 on surface ?s2
  (:action push
    :parameters (?a ?o ?p1 ?s1 ?p2 ?s2 ?t)
    :precondition (and (Arm ?a)
                       (Block ?o)
                       (AtPose ?o ?p1)
                       (On ?o ?s1)
                       (Movable ?o)
                       (HandEmpty)
                       (CanMove)
                       (Pose ?o ?p2)
                       (Supported ?o ?p2 ?s2)
                       (PushMotion ?o ?p1 ?t ?p2))
    :effect (and (AtPose ?o ?p2)
                 (On ?o ?s2)
                 (not (AtPose ?o ?p1))
                 (not (On ?o ?s1)))
  )

  (:derived (Movable ?o)
    (and (Block ?o) (not (exists (?o2) (On ?o2 ?o))))
  )
)

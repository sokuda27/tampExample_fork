(define (stream panda-tamp)
  (:stream sample-pose
    :inputs (?o ?r)
    :domain (Stackable ?o ?r)
    :outputs (?p)
    :certified (and (Pose ?o ?p) (Supported ?o ?p ?r))
  )

  (:stream plan-free-motion
    :inputs (?a ?q1 ?q2)
    :domain (and (Arm ?a) (Conf ?q1) (Conf ?q2))
    :outputs (?t)
    :certified (and (Traj ?t) (FreeMotion ?a ?q1 ?q2 ?t))
  )

  (:stream sample-push
    :inputs (?a ?o ?p1 ?p2)
    :domain (and (Arm ?a) (Pose ?o ?p1) (Pose ?o ?p2))
    :outputs (?t)
    :certified (and (Traj ?t) (PushMotion ?a ?o ?p1 ?p2 ?t))
  )

  (:stream test-pose-cfree
    :inputs (?o1 ?p1 ?o2 ?p2)
    :domain (and (Pose ?o1 ?p1) (Pose ?o2 ?p2))
    :certified (ObjCFreePose ?o1 ?p1 ?o2 ?p2)
  )

  (:stream test-traj-cfree
    :inputs (?a ?t ?o ?p)
    :domain (and (Arm ?a) (Traj ?t) (Pose ?o ?p))
    :certified (ObjCFreeTraj ?a ?t ?o ?p)
  )
)
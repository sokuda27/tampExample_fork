(define (stream block-pushing)
  ; Sample a stable pose for block ?o on the mat region ?s
  (:stream sample-push-pose
    :inputs (?o ?s)
    :domain (and (Block ?o) (Region ?s))
    :outputs (?p)
    :certified (and (Pose ?o ?p) (Supported ?o ?p ?s))
  )
  ; Plan a straight-line push trajectory moving block ?o from ?p1 to ?p2
  (:stream plan-push-motion
    :inputs (?a ?o ?p1 ?p2)
    :domain (and (Arm ?a) (Block ?o) (Pose ?o ?p1) (Pose ?o ?p2))
    :outputs (?t)
    :certified (PushMotion ?o ?p1 ?t ?p2)
  )
)

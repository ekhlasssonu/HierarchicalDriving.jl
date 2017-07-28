#import Base: ==, hash


type FSM_Node{T}
  nodeLabel::T
end
==(a::FSM_Node, b::FSM_Node) = (a.nodeLabel == b.nodeLabel)
Base.hash(a::FSM_Node, h::UInt64=zero(UInt64)) = hash(a.nodeLabel, h)
Base.copy(a::FSM_Node) = FSM_Node(a.nodeLabel)

type FSM_Edge{T}
  edgeLabel::T
end
==(a::FSM_Edge, b::FSM_Edge) = (a.edgeLabel == b.edgeLabel)
Base.hash(a::FSM_Edge,h::UInt64=zero(UInt64)) = hash(a.edgeLabel,h)
Base.copy(a::FSM_Edge) = FSM_Edge(a.edgeLabel)

type FSM{N,A,E}
  nodeSet::Array{FSM_Node{N},1}
  edgeLabels::Array{FSM_Edge{E},1}

  #Map nodes to distribution over actions
  actionProb::Dict{Tuple{FSM_Node{N}, A}, Float64}
  #Map node, edge, node to transition probability
  transitionProb::Dict{Tuple{FSM_Node{N}, FSM_Edge{E}, FSM_Node{N}}, Float64}
end
==(a::FSM, b::FSM) = (a.nodeSet == b.nodeSet) && (a.edgeLabels == b.edgeLabels) && (a.actionProb == b.actionProb) && (a.transitionProb == b.transitionProb)
Base.hash(a::FSM, h::UInt64=zero(UInt64)) = hash(a.nodeSet, hash(a.edgeLabels, hash(a.actionProb, hash(a.transitionProb,h))))
Base.copy(a::FSM) = FSM(a.nodeSet, a.edgeLabels, a.actionProb, a.transitionProb)

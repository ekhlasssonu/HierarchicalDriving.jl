#import Base: ==, hash


type FSM_Node{T}
  nodeLabel::T
end
=={T}(a::FSM_Node{T}, b::FSM_Node{T}) = (a.nodeLabel == b.nodeLabel)
Base.hash{T}(a::FSM_Node{T}, h::UInt64=zero(UInt64)) = hash(a.nodeLabel, h)
Base.copy{T}(a::FSM_Node{T}) = FSM_Node{T}(a.nodeLabel)

type FSM_Edge{T}
  edgeLabel::T
end
=={T}(a::FSM_Edge{T}, b::FSM_Edge{T}) = (a.edgeLabel == b.edgeLabel)
Base.hash{T}(a::FSM_Edge{T},h::UInt64=zero(UInt64)) = hash(a.edgeLabel,h)
Base.copy{T}(a::FSM_Edge{T}) = FSM_Edge{T}(a.edgeLabel)

type FSM{N,A,E}
  nodeSet::Array{FSM_Node{N},1}
  edgeLabels::Array{FSM_Edge{E},1}

  #Map nodes to distribution over actions
  actionProb::Dict{Tuple{FSM_Node{N}, A}, Float64}
  #Map node, edge, node to transition probability
  transitionProb::Dict{Tuple{FSM_Node{N}, FSM_Edge{E}, FSM_Node{N}}, Float64}
end
=={N,A,E}(a::FSM{N,A,E}, b::FSM{N,A,E}) = (a.nodeSet == b.nodeSet) && (a.nodeLabel == b.nodeLabel) && (a.actionProb == b.actionProb) && (a.transitionProb == b.transitionProb)
Base.hash{N,A,E}(a::FSM{N,A,E}, h::UInt64=zero(UInt64)) = hash(a.nodeSet, hash(a.edgeLabels, hash(actionProb, hash(transitionProb,h))))
Base.copy{N,A,E}(a::FSM{N,A,E}) = FSM{N,A,E}(a.nodeSet, a.edgeLabels, a.actionProb, a.transitionProb)

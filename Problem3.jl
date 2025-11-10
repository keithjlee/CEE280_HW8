using Asap, AsapOptim, Zygote
import Optimization as opt
using OptimizationOptimJL

function generate_cantilever(dxdy; E = 200e6, A = 1e-3, P = 50.0)

  #split input variable into dx, dy
  dx = dxdy[1]
  dy = dxdy[2]

  #make nodes
  n1 = TrussNode([0., 0., 0.], [false, false, false])
  n2 = TrussNode([2.0, 0., 0.], [true, true, false])
  n3 = TrussNode([4.0, 0., 0.], [true, true, false])
  n4 = TrussNode([0., -2.0, 0.], [false, false, false])
  n5 = TrussNode([2.0 + dx, -2.0 + dy, 0.], [true, true, false])

  nodes = [n1, n2, n3, n4, n5]

  #make elements
  section = TrussSection(A, E)

  e1 = TrussElement(n1, n2, section)
  e2 = TrussElement(n2, n3, section)
  e3 = TrussElement(n1, n5, section)
  e4 = TrussElement(n5, n2, section)
  e5 = TrussElement(n5, n3, section)
  e6 = TrussElement(n4, n5, section)

  elements = [e1, e2, e3, e4, e5, e6]

  #make loads
  load1 = NodeForce(n2, [0, -P, 0])
  load2 = NodeForce(n3, [0, -P, 0])

  loads = [load1, load2]

  #assemble model
  model = TrussModel(nodes, elements, loads)

  #solve for displacements
  Asap.solve!(model)

  return model

end

model = generate_cantilever([0, 0])


vars = [SpatialVariable(model.nodes[5], 0., -1., 1., :X), SpatialVariable(model.nodes[5], 0., -1., 1., :Y)]
params = TrussOptParams(model, vars)
x0 = params.values

function obj(x, p)
    res = solve_truss(x, p)
    F = AsapOptim.axial_force(res, p)

    return dot(abs.(F), res.L)
end

OBJ = x -> obj(x, params)
OBJ(x0)

callback = (state, loss)

optf = OptimizationFunction(obj, Optimization.AutoZygote())
prob = OptimizationProblem(optf, x0, params, lb = params.lb, ub = params.ub)


@time sol1 = opt.solve(prob, Optim.NelderMead())
OBJ(sol1.u)
@time sol2 = opt.solve(prob, Optim.LBFGS())
OBJ(sol2.u)


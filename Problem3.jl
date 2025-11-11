#=
Press shift+enter to run lines/blocks of code!
=#

#load the environment and install any packages as required
begin
    using Pkg
    Pkg.activate(".")
    Pkg.instantiate()
end

#define some utility functions and load packages
include("utilities.jl")

function portal(L, H, b, d)

    #node
    n1 = TrussNode([0., 0., 0.], [false, false, false])
    n2 = TrussNode([L, 0., 0.], [false, false, false])
    n3 = TrussNode([0., H, 0.], [true, true, false])

    dx = L / 3
    n4 = TrussNode([dx, H, 0.], [true, true, false])
    n5 = TrussNode([2dx, H, 0.], [true, true, false])
    n6 = TrussNode([L, H, 0.], [true, true, false])

    n7 = TrussNode([dx/2, H-d, 0.], [true, true, false])
    n8 = TrussNode([1.5dx, H-d, 0.], [true, true, false])
    n9 = TrussNode([2.5dx, H-d, 0.], [true, true, false])

    nodes = [n1, n2, n3, n4, n5, n6, n7, n8, n9]

    #elements
    A = .05 #m²
    E = 13e6 #kN/m²
    sec = TrussSection(A, E)

    connectivities = [
        [1,3],
        [1,7],
        [2,9],
        [2,6],
        [3,4],
        [4,5],
        [5,6],
        [7,8],
        [8,9],
        [3,7],
        [4,7],
        [4,8],
        [8,5],
        [5,9],
        [9,6]
    ]

    #columns
    elements = [TrussElement(nodes[c[1]], nodes[c[2]], sec) for c in connectivities]

    #loads
    P = [0., -25., 0.]
    loads = [NodeForce(node, P) for node in nodes[3:6]]

    #assemble
    model = TrussModel(nodes, elements, loads)
    Asap.solve!(model)

    return model
end

model = portal(20, 9, 2, 2)

pts, els = model_to_geometry(model)

begin
    fig = Figure()
    ax = Axis(
        fig[1,1],
        aspect = DataAspect()
    )

    linesegments!(els)
    scatter!(pts)

    fig
end



vars = [
    SpatialVariable(model.nodes[3], 0., -1., 2., :Y),
    SpatialVariable(model.nodes[4], 0., -1., 2., :Y),
    SpatialVariable(model.nodes[5], 0., -1., 2., :Y),
    SpatialVariable(model.nodes[6], 0., -1., 2., :Y),
    SpatialVariable(model.nodes[7], 0., 0., 1., :Y),
    SpatialVariable(model.nodes[8], 0., 0., 1., :Y),
    SpatialVariable(model.nodes[9], 0., 0., 1., :Y),
    SpatialVariable(model.nodes[7], 0., -1., 1., :X),
    SpatialVariable(model.nodes[9], 0., -1., 1., :X),
]

params = TrussOptParams(model, vars)
x0 = params.values

function obj(x, p)
    res = solve_truss(x, p)
    f = AsapOptim.axial_force(res, p)

    dot(p.P, res.U)
end


optf = OptimizationFunction(obj, Optimization.AutoZygote())
prob = OptimizationProblem(
    optf,
    x0,
    params,
    lb = params.lb,
    ub = params.ub
)

sol = Optimization.solve(prob, gradient_descent, maxiters = 3000, f_calls_limit = 30_000, store_trace = true)
sol.objective


model2 = updatemodel(params, sol.u)

pts, els = model_to_geometry(model2)

begin
    fig = Figure()
    ax = Axis(
        fig[1,1],
        aspect = DataAspect()
    )

    linesegments!(els)
    scatter!(pts)

    fig
end


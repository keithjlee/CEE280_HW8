# dependencies
using LinearAlgebra, Statistics
using Asap, AsapOptim, Zygote
using Optimization, OptimizationOptimJL
using CairoMakie

n_samples = 50
dx_range = range(-1, 1, n_samples)
dy_range = range(-1, 1, n_samples)

# Store optimization results
struct OptimizationResults
    x_opt
    f_opt
    x_history
    f_history
    time
end

# Perform optimization using Optimization.jl
function optimize(objective, params, x0, algorithm)

    x0 = Float64.(x0)
    #set up optimization function
    optf = OptimizationFunction(objective, Optimization.AutoZygote())

    #set up optimization problem
    prob = OptimizationProblem(
        optf,
        x0,
        params
    )

    #this tracks the history of the variables and objective
    local x_store = [x0]
    local f_store = [objective(x0, params)]
    callback = function(state, loss)
        push!(x_store, state.u)
        push!(f_store, loss)
        return false
    end

    #time and solve problem
    t0 = time()
    sol = Optimization.solve(prob, algorithm, callback = callback)
    dt = time() - t0


    println("OPTIMIZATION RESULTS")
    println("TOTAL TIME: $dt s")
    println("NUMBER OF ITERATIONS: $(length(x_store))")
    println("MINIMUM: $(sol.objective)")
    println("MINIMIZER: $(sol.u)")
    #
    return OptimizationResults(sol.u, sol.objective, x_store, f_store, dt);

end

# Make an asap model of the cantilever
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

generate_cantilever(dx, dy) = generate_cantilever([dx, dy])

function model_to_geometry(model::TrussModel)
    pts = Point2.(getproperty.(model.nodes, :position))
    els = Point2.(vcat([[e.nodeStart.position, e.nodeEnd.position] for e in model.elements]...))

    return pts, els
end

function draw_cantilever(x)
    model = generate_cantilever(x)

    pts, els = model_to_geometry(model)

    #limits
    xrange = extrema(getindex.(pts, 1)) .+ [-.5, .5]
    yrange = extrema(getindex.(pts, 2)) .+ [-.5, .5]

    fig = Figure()
    ax = Axis(
        fig[1,1],
        aspect = DataAspect(),
        limits = (xrange..., yrange...),
        xticksvisible = false,
        yticksvisible = false
    )

    hidespines!(ax)

    linesegments!(
        els,
        color = :black,
        linewidth = 2
    )

    scatter!(
        pts,
        color = :white,
        strokecolor = :black,
        strokewidth = 2
    )

    return fig
end

draw_cantilever(dx, dy) = draw_cantilever([dx, dy])

function generate_bridge(span, n_segments, stiffness_ratio = 2; include_cables = true)

    L = span
    n = n_segments
    dx = L / (n-1)

    xtop = collect(0:dx:L)
    xbot = [0; collect(dx/2:dx:L-dx/2); L]

    cable_offset = 3
    cable_points = .1 .* (xtop .- L / 2).^2 .+ cable_offset

    Atruss = 2e-3
    truss_section = TrussSection(Atruss, 200e6)
    cable_section = TrussSection(Atruss / stiffness_ratio, 200e6)

    d = 1.

    top_nodes = [TrussNode([x, 0., 0.], [true, true, false], :top) for x in xtop]
    top_nodes[1].dof = [false, false, false]
    top_nodes[end].dof = [false, false, false]

    bot_nodes = [TrussNode([x, -d, 0.], [true, true, false], :bot) for x in xbot]
    bot_nodes[1].dof = [false, false, false]
    bot_nodes[end].dof = [false, false, false]

    cable_nodes = [TrussNode([x, y, 0], [true, true, false], :cable) for (x,y) in zip(xtop, cable_points)]
    cable_nodes[1].dof = [false, false, false]
    cable_nodes[end].dof = [false, false, false]

    #elements
    top_chord = [TrussElement(top_nodes[i], top_nodes[i+1], truss_section) for i = 1:n-1]
    bottom_chord = [TrussElement(bot_nodes[i], bot_nodes[i+1], truss_section) for i = 1:n]

    web_elements = [
        [TrussElement(top_nodes[i], bot_nodes[i+1], truss_section) for i = 1:n-1];
        [TrussElement(bot_nodes[i], top_nodes[i], truss_section) for i = 2:n]
    ]

    #cable
    hanger_elements = [TrussElement(top_nodes[i], cable_nodes[i], cable_section, :cable) for i = 2:n-1]
    cable_elements = [TrussElement(cable_nodes[i], cable_nodes[i+1], cable_section, :cable) for i = 1:n-1]


    #loads
    loads = [NodeForce(node, [0., -10., 0.]) for node in top_nodes[2:end-1]]

    nodes = include_cables ? [top_nodes; bot_nodes; cable_nodes] : [top_nodes; bot_nodes]
    elements = include_cables ? [top_chord; bottom_chord; web_elements; hanger_elements; cable_elements] : [top_chord; bottom_chord; web_elements]

    model = TrussModel(nodes, elements, loads)
    Asap.solve!(model)

    return model
end

function FL(x, p)
    res = solve_truss(x, p)
    F = AsapOptim.axial_force(res, p)

    return dot(abs.(F), res.L)
end

function obj_compliance(x, p)
    res = solve_truss(x, p)
    return dot(res.U, p.P)
end
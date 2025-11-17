include("utilities.jl")

#=
HYPERPARAMETERS
=#
L = 20.
n = 11
bridge_to_cable_stiffness_ratio = 1


#generate bridge model
model = generate_bridge(L, n, bridge_to_cable_stiffness_ratio)

#visualize initial model
begin
    pts, els = model_to_geometry(model)

    el_colors = [e.id == :cable ? :gray : :black for e in model.elements]

    fig = Figure()
    ax = Axis(
        fig[1,1],
        aspect = DataAspect(),
        title = "Initial"
    )

    hidespines!(ax)

    yextrema = extrema(getindex.(pts, 2)) .+ (-.5, .5)
    ylims!(yextrema...)

    linesegments!(els, color = el_colors)
    scatter!(pts, color = :white, strokecolor = :black, strokewidth = 1)
    fig
end

#compute ∑|F|L for initial structure
FL_initial = dot(abs.(Asap.axial_force.(model.elements)), length.(model.elements))

#=
OPTIMIZATION
=#

#ALGORITHM OPTIONS
begin
  #Gradient free methods
  nelder_mead = Optim.NelderMead()

  #evolutionary algorithms
  particle_swarm = Optim.ParticleSwarm()

  #gradient-based algorithms
  gradient_descent = Optim.GradientDescent()
  conjugate_gradient_descent = Optim.ConjugateGradient()
  LBFGS = Optim.LBFGS()
end

#CHOOSE ALGORITHM
algorithm = LBFGS

#PERFORM OPTIMIZATION
begin
    vars = [
        [SpatialVariable(node, 0., -3., 0.5, :Y) for node in model.nodes[:bot]];
        [SpatialVariable(node, 0., -node.position[2] + .5, 5., :Y) for node in model.nodes[:cable]]
    ]

    params = TrussOptParams(model, vars)
    x0 = params.values

    optf = OptimizationFunction(FL, AutoZygote())
    prob = OptimizationProblem(
        optf,
        x0,
        params,
        lb = params.lb,
        ub = params.ub
    )

    println("OPTIMIZING")
    res = Optimization.solve(prob, algorithm, maxtime = 60, reltol = 1e-4)
end

#POST-PROCESS
model_opt = updatemodel(params, res.u)

#Objective value of optimal design
FL_optimal = FL(res.u, params)

#Visualize optimal design
begin
    pts_opt, els_opt = model_to_geometry(model_opt)

    fig = Figure()
    ax = Axis(
        fig[1,1],
        aspect = DataAspect(),
        title = "Optimal"
    )

    hidespines!(ax)

    linesegments!(els, color = (:black, .2), linestyle= :dash, linewidth = 1)

    yextrema = extrema([getindex.(pts_opt, 2); getindex.(pts, 2)]) .+ (-.5, .5)
    ylims!(yextrema...)

    linesegments!(els_opt, color = el_colors)
    scatter!(pts_opt, color = :white, strokecolor = :black, strokewidth = 1)
    fig
end
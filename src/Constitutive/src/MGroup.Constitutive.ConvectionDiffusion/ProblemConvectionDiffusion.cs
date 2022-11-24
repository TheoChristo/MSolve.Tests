using System.Collections.Generic;
using MGroup.MSolve.AnalysisWorkflow;
using MGroup.MSolve.AnalysisWorkflow.Providers;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization.BoundaryConditions;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.LinearSystem;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.Constitutive.ConvectionDiffusion.Providers;
using System.Linq;
using MGroup.Constitutive.ConvectionDiffusion.BoundaryConditions;


namespace MGroup.Constitutive.ConvectionDiffusion
{
	public class ProblemConvectionDiffusion : IAlgebraicModelInterpreter, ITransientAnalysisProvider, INonTransientAnalysisProvider, INonLinearProvider
	{
		private bool shouldRebuildDiffusionMatrixForZeroOrderDerivativeMatrixVectorProduct = false;
		private IGlobalMatrix convection, diffusion, production, capacityMatrix;
		private readonly IModel model;
		private readonly IAlgebraicModel algebraicModel;
		private readonly ISolver solver;
		private ElementProductionProvider productionProvider = new ElementProductionProvider();
		private ElementIndependentProductionProvider independentProductionProvider = new ElementIndependentProductionProvider();
		private ElementDiffusionProvider diffusionProvider = new ElementDiffusionProvider();
		private ElementConvectionProvider convectionProvider = new ElementConvectionProvider();
		private ElementCapacityMatrixProvider fistTimeDerivativeMatrixProvider = new ElementCapacityMatrixProvider();

		// TODO: Is this right?
		private readonly IElementMatrixPredicate rebuildDiffusionPredicate = new MaterialModifiedElementMarixPredicate();

		public ProblemConvectionDiffusion(IModel model, IAlgebraicModel algebraicModel, ISolver solver)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.solver = solver;
			algebraicModel.BoundaryConditionsInterpreter = this;

			ActiveDofs.AddDof(ConvectionDiffusionDof.UnknownVariable);
		}

		private IGlobalMatrix Convection
		{
			get
			{
				if (convection == null) BuildConvection();
				return convection;
			}
		}

		private IGlobalMatrix Diffusion
		{
			get
			{
				if (diffusion == null) BuildDiffusion();
				return diffusion;
			}
		}

		private IGlobalMatrix Production
		{
			get
			{
				if (production == null) BuildProduction();
				return production;
			}
		}

		private IGlobalMatrix CapacityMatrix
		{
			get
			{
				if (capacityMatrix == null) BuildCapacityMatrix();
				return capacityMatrix;
			}
		}

		public ActiveDofs ActiveDofs { get; } = new ActiveDofs();

		// Re malakes, eleos me to copy paste
		// All work and no play make jack a dull boy
		private void BuildConvection() => convection = algebraicModel.BuildGlobalMatrix(convectionProvider);

		// Re malakes, eleos me to copy paste
		// All work and no play make jack a dull boy
		private void BuildDiffusion() => diffusion = algebraicModel.BuildGlobalMatrix(diffusionProvider);

		// Re malakes, eleos me to copy paste
		// All work and no play make jack a dull bo y
		private void BuildProduction() => production = algebraicModel.BuildGlobalMatrix(productionProvider);

		// Re malakes, eleos me to copy paste
		// All work and no play make jack a  boy dull
		private void BuildCapacityMatrix() => capacityMatrix = algebraicModel.BuildGlobalMatrix(fistTimeDerivativeMatrixProvider);

		// TODO:Is this right?
		private void RebuildDiffusion() => algebraicModel.RebuildGlobalMatrixPartially(diffusion, 
			model.EnumerateElements, diffusionProvider, rebuildDiffusionPredicate);

		#region IAnalyzerProvider Members
		public void ClearMatrices()
		{
			convection = null;
			diffusion = null;
			production = null;
			capacityMatrix = null;
		}

		public void Reset()
		{
			ClearMatrices();
		}

		public void GetProblemDofTypes()
		{
			// Contents were commented out, function no longer used
		}
		#endregion

		#region IImplicitIntegrationProvider Members

		public void LinearCombinationOfMatricesIntoEffectiveMatrix(TransientAnalysisCoefficients coefficients)
		{
			IGlobalMatrix matrix = Diffusion;
			matrix.AddIntoThis(Convection);
			matrix.AddIntoThis(Production);
			matrix.AxpyIntoThis(CapacityMatrix, coefficients.FirstOrderDerivativeCoefficient);
			solver.LinearSystem.Matrix = matrix;
			shouldRebuildDiffusionMatrixForZeroOrderDerivativeMatrixVectorProduct = true;
		}

		public void LinearCombinationOfMatricesIntoEffectiveMatrixNoOverwrite(TransientAnalysisCoefficients coefficients)
		{
			IGlobalMatrix matrix = Diffusion.Copy();
			matrix.AddIntoThis(Convection);
			matrix.AddIntoThis(Production);
			matrix.AxpyIntoThis(CapacityMatrix, coefficients.FirstOrderDerivativeCoefficient);
			solver.LinearSystem.Matrix = matrix;
		}

		public void ProcessRhs(TransientAnalysisCoefficients coefficients, IGlobalVector rhs)
		{
			// Method intentionally left empty.
		}

		public IGlobalVector GetSecondOrderDerivativeVectorFromBoundaryConditions(double time) => algebraicModel.CreateZeroVector();

		public IGlobalVector GetFirstOrderDerivativeVectorFromBoundaryConditions(double time)
		{
			var boundaryConditions = model.EnumerateBoundaryConditions(model.EnumerateSubdomains().First().ID).ToArray();
			foreach (var boundaryCondition in boundaryConditions.OfType<ITransientBoundaryConditionSet<IConvectionDiffusionDofType>>())
			{
				boundaryCondition.CurrentTime = time;
			}

			IGlobalVector velocities = algebraicModel.CreateZeroVector();
			algebraicModel.AddToGlobalVector(
			id =>
			{
				var boundaryConditions = model.EnumerateBoundaryConditions(id);
				foreach (var boundaryCondition in boundaryConditions.OfType<ITransientBoundaryConditionSet<IConvectionDiffusionDofType>>().ToArray())
				{
					boundaryCondition.CurrentTime = time;
				}

				// TODO: Is this right?
				return boundaryConditions
					.SelectMany(x => x.EnumerateNodalBoundaryConditions())
					.OfType<INodalCapacityBoundaryCondition>();
			},
			velocities);

			// TODO: Is this right?
			algebraicModel.AddToGlobalVector(boundaryConditions
				.SelectMany(x => x.EnumerateDomainBoundaryConditions())
				.OfType<IDomainCapacityBoundaryCondition>(),
			velocities);

			return velocities;
		}

		public IGlobalVector GetRhs(double time)
		{
			solver.LinearSystem.RhsVector.Clear();
			AssignRhs();
			IGlobalVector result = solver.LinearSystem.RhsVector.Copy();

			return result;
		}

		public IGlobalVector SecondOrderDerivativeMatrixVectorProduct(IGlobalVector vector) => algebraicModel.CreateZeroVector();

		public IGlobalVector FirstOrderDerivativeMatrixVectorProduct(IGlobalVector vector)
		{// TODO: is the matrix right?
			IGlobalVector result = algebraicModel.CreateZeroVector();
			CapacityMatrix.MultiplyVector(vector, result);
			return result;
		}

		public IGlobalVector ZeroOrderDerivativeMatrixVectorProduct(IGlobalVector vector)
		{// TODO: is the matrix right?
			if (shouldRebuildDiffusionMatrixForZeroOrderDerivativeMatrixVectorProduct)
			{
				BuildDiffusion();
				shouldRebuildDiffusionMatrixForZeroOrderDerivativeMatrixVectorProduct = false;
			}

			IGlobalVector result = algebraicModel.CreateZeroVector();
			//Needs fix for diffusion matrx to exist (I suppose)
			//DiffusionMatrix.MultiplyVector(vector, result);

			diffusion.MultiplyVector(vector, result); // TODO: Is that what george wanted?
			
			return result;
		}

	#endregion

	#region IStaticProvider Members

		public void CalculateMatrix()
		{
			if (diffusion == null) BuildDiffusion();
			if (convection == null) BuildConvection();
			if (production == null) BuildProduction();

			var effectiveMatrix = diffusion.Copy();
			effectiveMatrix.AddIntoThis(convection);
			effectiveMatrix.AddIntoThis(production);
			solver.LinearSystem.Matrix = effectiveMatrix;
		}
		#endregion

		#region INonLinearProvider Members

		public double CalculateRhsNorm(IGlobalVector rhs) => rhs.Norm2();

		public void ProcessInternalRhs(IGlobalVector solution, IGlobalVector rhs) { }

		#endregion

		public void AssignRhs()
		{
			solver.LinearSystem.RhsVector.Clear();

			algebraicModel.AddToGlobalVector(solver.LinearSystem.RhsVector, independentProductionProvider);

			algebraicModel.AddToGlobalVector(id =>
				model.EnumerateBoundaryConditions(id)
					.SelectMany(x => x.EnumerateNodalBoundaryConditions())
					.OfType<INodalUnknownVariableFluxBoundaryCondition>()
					.Where(x => model.EnumerateBoundaryConditions(id)
						.SelectMany(x => x.EnumerateNodalBoundaryConditions())
						.OfType<INodalUnknownVariableBoundaryCondition>()
						.Any(d => d.Node.ID == x.Node.ID && d.DOF == x.DOF) == false), 
					solver.LinearSystem.RhsVector);
		}

		public IEnumerable<INodalNeumannBoundaryCondition<IDofType>> EnumerateEquivalentNeumannBoundaryConditions(int subdomainID) =>
			model.EnumerateBoundaryConditions(subdomainID)
				.SelectMany(x => x.EnumerateEquivalentNodalNeumannBoundaryConditions(model.EnumerateElements(subdomainID)))
				.OfType<INodalUnknownVariableFluxBoundaryCondition>()
				.Where(x => model.EnumerateBoundaryConditions(subdomainID)
					.SelectMany(x => x.EnumerateNodalBoundaryConditions())
					.OfType<INodalUnknownVariableBoundaryCondition>()
					.Any(d => d.Node.ID == x.Node.ID && d.DOF == x.DOF) == false);

		public IDictionary<(int, IDofType), (int, INode, double)> GetDirichletBoundaryConditionsWithNumbering() =>
			model.EnumerateSubdomains()
				.SelectMany(x => model.EnumerateBoundaryConditions(x.ID)
					.SelectMany(x => x.EnumerateNodalBoundaryConditions()).OfType<INodalUnknownVariableBoundaryCondition>()
					.OrderBy(x => x.Node.ID)
					.GroupBy(x => (x.Node.ID, x.DOF))
					.Select((x, Index) => (x.First().Node, (IDofType)x.Key.DOF, Index, x.Sum(a => a.Amount))))
				.ToDictionary(x => (x.Node.ID, x.Item2), x => (x.Index, x.Node, x.Item4));

		public IDictionary<(int, IDofType), (int, INode, double)> GetDirichletBoundaryConditionsWithNumbering(int subdomainID) =>
			model.EnumerateBoundaryConditions(subdomainID)
				.SelectMany(x => x.EnumerateNodalBoundaryConditions()).OfType<INodalUnknownVariableBoundaryCondition>()
				.OrderBy(x => x.Node.ID)
				.GroupBy(x => (x.Node.ID, x.DOF))
				.Select((x, Index) => (x.First().Node, (IDofType)x.Key.DOF, Index, x.Sum(a => a.Amount)))
				.ToDictionary(x => (x.Node.ID, x.Item2), x => (x.Index, x.Node, x.Item4));
	}
}

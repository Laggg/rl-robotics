using Unity.MLAgents;
using UnityEngine;

/// <summary>
/// Custom decision requester that overrides decision period with a hyperparameter provided in the python config file
/// (through the academy environment parameters)
/// </summary>
    
[RequireComponent(typeof(Agent))]
public class CustomDecisionRequester : MonoBehaviour
{
    [Tooltip("The frequency with which the agent requests a decision. A DecisionPeriod " +
             "of 10 means that the Agent will request a decision every 10 AcademySteps (FixedUpdates) or 5 timesPerSecond")]
    [SerializeField] private int decisionPeriod = 10;
    public int DecisionPeriod => decisionPeriod;

    [Tooltip("Indicates whether or not the agent will take an action during the Academy " +
             "steps where it does not request a decision. Has no effect when DecisionPeriod " +
             "is set to 1.")]
    [SerializeField] private  bool takeActionsBetweenDecisions = false;
        
    private Agent agent;
        
    private void Awake()
    {
        agent = GetComponent<Agent>();
        decisionPeriod = (int)Academy.Instance.EnvironmentParameters.GetWithDefault("decision_period", decisionPeriod);
        // Debug.Log($"Decision period is {decisionPeriod}");
        Academy.Instance.AgentPreStep += MakeRequests;
    }

    private void OnDestroy()
    {
        if (Academy.IsInitialized)
        {
            Academy.Instance.AgentPreStep -= MakeRequests;
        }
    }

    /// <summary>
    /// Information about Academy step used to make decisions about whether to request a decision.
    /// </summary>
    private struct DecisionRequestContext
    {
        /// <summary>
        /// The current step count of the Academy, equivalent to Academy.StepCount.
        /// </summary>
        public int AcademyStepCount;
    }

    /// <summary>
    /// Method that hooks into the Academy in order inform the Agent on whether or not it should request a
    /// decision, and whether or not it should take actions between decisions.
    /// </summary>
    /// <param name="academyStepCount">The current step count of the academy.</param>
    private void MakeRequests(int academyStepCount)
    {
        var context = new DecisionRequestContext
        {
            AcademyStepCount = academyStepCount
        };

        if (ShouldRequestDecision(context))
        {
            agent.RequestDecision();
        }

        if (ShouldRequestAction(context))
        {
            agent.RequestAction();
        }
    }

    /// <summary>
    /// Whether Agent.RequestDecision should be called on this update step.
    /// </summary>
    /// <param name="context"></param>
    /// <returns></returns>
    private bool ShouldRequestDecision(DecisionRequestContext context)
    {
        return context.AcademyStepCount % decisionPeriod == 0;
    }

    /// <summary>
    /// Whether Agent.RequestAction should be called on this update step.
    /// </summary>
    /// <param name="context"></param>
    /// <returns></returns>
    private bool ShouldRequestAction(DecisionRequestContext context)
    {
        return takeActionsBetweenDecisions;
    }
}
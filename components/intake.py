if True:
    from . import intake_v1

    Intake = intake_v1.Intake
    ActionIntakeEntree = intake_v1.ActionIntakeEntree
    ActionIntakeSortie = intake_v1.ActionIntakeSortie
else:
    from . import intake_v2

    Intake = intake_v2.Intake
    ActionIntakeEntree = intake_v2.ActionIntakeEntree
    ActionIntakeSortie = intake_v2.ActionIntakeSortie

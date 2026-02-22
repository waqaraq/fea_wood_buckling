# =========================
# Parameterized ABAQUS script (function-based inputs)
# =========================

from abaqus import mdb
from abaqusConstants import *
from part import *
from material import *
from section import *
from assembly import *
from step import *
from load import *
from mesh import *
from job import *
from sketch import *
from connectorBehavior import *
from regionToolset import Region
from odbAccess import openOdb

EIGEN_RESULTS = {}


def build_beam_buckle_model(length, radius, ch, cv, E, nu,
                            left_bc=2, right_bc=4,
                            model_name='Model-1',
                            job_name='EVB_Job1'):
    """
    
    # ============================================================
    # INPUTS (from function)
    # ============================================================
    BEAM_LENGTH = float(length)
    RADIUS      = float(radius)
    Ch          = float(ch)   # dof=2 stiffness
    Cv          = float(cv)   # dof=1 stiffness
    E_MOD       = float(E)
    NU          = float(nu)

    left_bc  = int(left_bc)
    right_bc = int(right_bc)

    # ============================================================
    # EVERYTHING ELSE (names, mesh, step, job, BCs, loads, etc.)
    # ============================================================
    MODEL_NAME = model_name

    # Names
    PART_NAME     = 'Beam'
    INSTANCE_NAME = PART_NAME + '-1'
    SKETCH_NAME   = '__profile__'

    MAT_NAME     = 'Steel'
    PROFILE_NAME = 'Beam_Cross_Section'
    SECTION_NAME = 'Beam_Section'

    # Derived geometry
    BEAM_START = (0.0, 0.0)
    BEAM_END   = (BEAM_LENGTH, 0.0)

    # Section orientation / options
    BEAM_N1               = (0.0, 0.0, -1.0)
    SECTION_POISSON_RATIO = 0.0  # kept same as your original script

    # Step (Buckling)
    STEP_NAME      = 'EVB'
    NUM_EIGEN      = 1
    VECTORS        = 20
    MAX_ITERATIONS = 3000

    # Mesh
    MESH_SIZE        = 1.0
    DEVIATION_FACTOR = 0.1
    MIN_SIZE_FACTOR  = 0.1
    NODE_SET_NAME    = 'ALL_NODES'

    # Springs names (feature names in Abaqus)
    SPRING_NAME_DOF2 = 'Ch'
    SPRING_NAME_DOF1 = 'Cv'

    # Loads / BCs
    AXIAL_LOAD_NAME = 'Axial_Load'
    AXIAL_FORCE_CF1 = 1000.0

    BC_LEFT_NAME  = 'BC-Left'
    BC_RIGHT_NAME = 'BC-Right'

    # Job
    JOB_NAME    = job_name
    MEMORY_PCT  = 90
    NUM_CPUS    = 1
    NUM_GPUS    = 0
    NUM_THREADS = 1

    # -------------------------
    # BUILD / RESET MODEL
    # -------------------------
    # If the model doesn't exist, create it. If it exists, reuse it.
    if MODEL_NAME not in mdb.models.keys():
        mdb.Model(name=MODEL_NAME)

    model = mdb.models[MODEL_NAME]

    # -------------------------
    # Sketch (line)
    # -------------------------
    sheet_size = max(1000.0, 5.0 * BEAM_LENGTH)
    sk = model.ConstrainedSketch(name=SKETCH_NAME, sheetSize=sheet_size)
    sk.Line(point1=BEAM_END, point2=BEAM_START)

    sk.ObliqueDimension(
        textPoint=(0.45 * BEAM_LENGTH, -0.12 * BEAM_LENGTH),
        value=BEAM_LENGTH,
        vertex1=sk.vertices[0],
        vertex2=sk.vertices[1]
    )

    # -------------------------
    # Part
    # -------------------------
    # If part exists, delete it to rebuild cleanly
    if PART_NAME in model.parts.keys():
        del model.parts[PART_NAME]

    part = model.Part(name=PART_NAME, dimensionality=TWO_D_PLANAR, type=DEFORMABLE_BODY)
    part.BaseWire(sketch=sk)
    del model.sketches[SKETCH_NAME]

    # -------------------------
    # Material
    # -------------------------
    if MAT_NAME not in model.materials.keys():
        model.Material(name=MAT_NAME)
    model.materials[MAT_NAME].Elastic(table=((E_MOD, NU), ))

    # -------------------------
    # Profile + section
    # -------------------------
    if PROFILE_NAME not in model.profiles.keys():
        model.CircularProfile(name=PROFILE_NAME, r=RADIUS)
    else:
        model.profiles[PROFILE_NAME].setValues(r=RADIUS)

    # If section exists, delete and recreate (avoids conflicts when rerunning)
    if SECTION_NAME in model.sections.keys():
        del model.sections[SECTION_NAME]

    model.BeamSection(
        name=SECTION_NAME,
        integration=DURING_ANALYSIS,
        material=MAT_NAME,
        profile=PROFILE_NAME,
        poissonRatio=SECTION_POISSON_RATIO,
        temperatureVar=LINEAR,
        consistentMassMatrix=False,
        beamSectionOffset=(0.0, 0.0)
    )

    # Assign section to the (only) edge(s)
    edge_region = Region(edges=part.edges[:])
    part.SectionAssignment(
        region=edge_region,
        sectionName=SECTION_NAME,
        offset=0.0,
        offsetType=MIDDLE_SURFACE,
        offsetField='',
        thicknessAssignment=FROM_SECTION
    )
    part.assignBeamSectionOrientation(
        region=edge_region,
        method=N1_COSINES,
        n1=BEAM_N1
    )

    # -------------------------
    # Assembly + instance
    # -------------------------
    asm = model.rootAssembly
    asm.DatumCsysByDefault(CARTESIAN)

    # If instance exists, delete it (safe rebuild)
    if INSTANCE_NAME in asm.instances.keys():
        del asm.instances[INSTANCE_NAME]

    asm.Instance(name=INSTANCE_NAME, part=part, dependent=ON)

    # -------------------------
    # Step
    # -------------------------
    if STEP_NAME in model.steps.keys():
        del model.steps[STEP_NAME]

    model.BuckleStep(
        name=STEP_NAME,
        previous='Initial',
        numEigen=NUM_EIGEN,
        vectors=VECTORS,
        maxIterations=MAX_ITERATIONS
    )

    # -------------------------
    # Mesh + node set (all nodes)
    # -------------------------
    part.seedPart(size=MESH_SIZE, deviationFactor=DEVIATION_FACTOR, minSizeFactor=MIN_SIZE_FACTOR)
    part.generateMesh()

    if NODE_SET_NAME in part.sets.keys():
        del part.sets[NODE_SET_NAME]
    part.Set(name=NODE_SET_NAME, nodes=part.nodes[:])

    asm.regenerate()

    # -------------------------
    # Springs to ground (on all nodes)
    # -------------------------
    node_region = asm.instances[INSTANCE_NAME].sets[NODE_SET_NAME]

    # If springs exist, delete them (safe rebuild)
    if SPRING_NAME_DOF2 in asm.engineeringFeatures.springDashpots.keys():
        del asm.engineeringFeatures.springDashpots[SPRING_NAME_DOF2]
    if SPRING_NAME_DOF1 in asm.engineeringFeatures.springDashpots.keys():
        del asm.engineeringFeatures.springDashpots[SPRING_NAME_DOF1]

    asm.engineeringFeatures.SpringDashpotToGround(
        name=SPRING_NAME_DOF2,
        region=node_region,
        dof=2,
        springBehavior=ON,
        springStiffness=Ch,
        dashpotBehavior=OFF,
        dashpotCoefficient=0.0,
        orientation=None
    )

    asm.engineeringFeatures.SpringDashpotToGround(
        name=SPRING_NAME_DOF1,
        region=node_region,
        dof=1,
        springBehavior=ON,
        springStiffness=Cv,
        dashpotBehavior=OFF,
        dashpotCoefficient=0.0,
        orientation=None
    )

    # -------------------------
    # Loads + boundary conditions
    # -------------------------
    inst = asm.instances[INSTANCE_NAME]
    v_left  = inst.vertices.findAt(((BEAM_START[0], BEAM_START[1], 0.0),))
    v_right = inst.vertices.findAt(((BEAM_END[0],   BEAM_END[1],   0.0),))

    # Remove existing loads/BCs with same names (safe rerun)
    if AXIAL_LOAD_NAME in model.loads.keys():
        del model.loads[AXIAL_LOAD_NAME]
    if BC_LEFT_NAME in model.boundaryConditions.keys():
        del model.boundaryConditions[BC_LEFT_NAME]
    if BC_RIGHT_NAME in model.boundaryConditions.keys():
        del model.boundaryConditions[BC_RIGHT_NAME]

    # Load stays applied on the LEFT
    model.ConcentratedForce(
        name=AXIAL_LOAD_NAME,
        createStepName=STEP_NAME,
        region=Region(vertices=v_left),
        cf1=AXIAL_FORCE_CF1,
        distributionType=UNIFORM,
        field='',
        localCsys=None
    )

    # BC at LEFT (option 1/2)
    if left_bc == 1:
        pass  # Left_free: no BC
    elif left_bc == 2:
        # Left_pinned: U2 = 0 only
        model.DisplacementBC(
            name=BC_LEFT_NAME,
            createStepName='Initial',
            region=Region(vertices=v_left),
            u1=UNSET, u2=SET, ur3=UNSET,
            amplitude=UNSET,
            distributionType=UNIFORM,
            fieldName='',
            localCsys=None
        )
    else:
        raise ValueError('left_bc must be 1 (Left_free) or 2 (Left_pinned)')

    # BC at RIGHT (option 3/4)
    if right_bc == 3:
        pass  # Right_free: no BC
    elif right_bc == 4:
        # Right_pinned: U1 = U2 = 0
        model.DisplacementBC(
            name=BC_RIGHT_NAME,
            createStepName='Initial',
            region=Region(vertices=v_right),
            u1=SET, u2=SET, ur3=UNSET,
            amplitude=UNSET,
            distributionType=UNIFORM,
            fieldName='',
            localCsys=None
        )
    else:
        raise ValueError('right_bc must be 3 (Right_free) or 4 (Right_pinned)')

    # -------------------------
    # Job
    # -------------------------
    # If job exists, delete it
    if JOB_NAME in mdb.jobs.keys():
        del mdb.jobs[JOB_NAME]

    mdb.Job(
        name=JOB_NAME,
        model=MODEL_NAME,
        type=ANALYSIS,
        memory=MEMORY_PCT,
        memoryUnits=PERCENTAGE,
        numCpus=NUM_CPUS,
        numGPUs=NUM_GPUS,
        numThreadsPerMpiProcess=NUM_THREADS,
        multiprocessingMode=DEFAULT,
        resultsFormat=ODB,
        explicitPrecision=SINGLE,
        nodalOutputPrecision=SINGLE,
        echoPrint=OFF,
        modelPrint=OFF,
        contactPrint=OFF,
        historyPrint=OFF,
        description='',
        scratch='',
        userSubroutine='',
        waitHours=0,
        waitMinutes=0,
        getMemoryFromAnalysis=True
    )

    mdb.jobs[JOB_NAME].submit(consistencyChecking=OFF)
    mdb.jobs[JOB_NAME].waitForCompletion()
    odb = openOdb(path=JOB_NAME + '.odb')
    step = odb.steps[STEP_NAME]
    if len(step.frames) >= 2:
        frame = step.frames[1]
    else:
        frame = step.frames[-1]
    desc = frame.description
    idx = desc.lower().find('eigenvalue')
    if idx != -1 and '=' in desc[idx:]:
        eigenvalue = float(desc[idx:].split('=')[1].strip().split()[0])
    else:
        eigenvalue = frame.frameValue
    odb.close()
    buckling_load = eigenvalue
    EIGEN_RESULTS[JOB_NAME] = eigenvalue

    return model


# -------------------------
# Example call (edit values as needed)
# -------------------------
cases = [
    ('Pinned-Pinned', 2, 4, 'PP'),
]

tests = [
    ('HT-T 8.0x160', 430, 160, 8, 5.7),
    ('HT-T 8.0x180', 430, 180, 8, 5.7),
    ('HT-T 8.0x200', 430, 200, 8, 5.7),
    ('HT-T 8.0x300', 430, 300, 8, 5.7),
    ('HT-T 8.0x340', 430, 340, 8, 5.7),
    ('VGZ 9.0x440',  430, 440, 9, 6.4),
    ('NMBU 8x200',   469, 200, 8, 5.5),
    ('NMBU 8x220',   469, 220, 8, 5.5),
    ('NMBU 8x260',   469, 260, 8, 5.5),
    ('NMBU 8x280',   469, 280, 8, 5.5),
    ('NMBU 10x300',  469, 300, 10, 6.25),
    ('NMBU 10x340',  469, 340, 10, 6.25),
    ('KIT-L 6x200',  394, 200, 6, 3.9),
    ('KIT-L 8x260',  394, 260, 8, 5.5),
    ('KIT-L 10x300', 394, 300, 10, 6.25),
    ('KIT-H 6x200',  536, 200, 6, 3.9),
    ('KIT-H 8x260',  536, 260, 8, 5.5),
    ('KIT-H 10x300', 536, 300, 10, 6.25),
]

NUM_TESTS_TO_RUN = 18

results = {}
for t_i, (test_name, test_density, test_length, test_d, test_d1) in enumerate(tests[:NUM_TESTS_TO_RUN]):
    results[test_name] = {}

    ch = (0.19 + 0.012 * float(test_d)) * float(test_density)
    ch = ch * float(test_d1) / (2*float(test_d))
    cv = 234.0 * (float(test_density) * float(test_d)) ** 0.2 / (float(test_length) ** 0.6)

    results[test_name]['Ch'] = ch
    results[test_name]['Cv'] = cv

    # Only PP runs (single case in "cases")
    for label, lbc, rbc, suffix in cases:
        jn = 'EVB_Job1_T%02d_%s' % (t_i + 1, suffix)   # EVB_Job1_T01_PP ... T18_PP

        build_beam_buckle_model(
            length=test_length,
            radius=float(test_d1) / 2.0,
            ch=ch,
            cv=cv,
            E=210000.0,
            nu=0.30,
            left_bc=lbc,       # 2
            right_bc=rbc,      # 4
            job_name=jn
        )
        results[test_name][label] = EIGEN_RESULTS[jn]

out = open('EVB_Job1_buckling_load_PP_only.txt', 'w')
out.write('Test\tDensity\tLength\tNominal Dia(d)\tCore dia(d1)\tCh\tCv\tPinned-Pinned\n')

for (test_name, test_density, test_length, test_d, test_d1) in tests[:NUM_TESTS_TO_RUN]:
    out.write(
        str(test_name) + '\t' +
        str(test_density) + '\t' +
        str(test_length) + '\t' +
        str(test_d) + '\t' +
        str(test_d1) + '\t' +
        str(results[test_name]['Ch']) + '\t' +
        str(results[test_name]['Cv']) + '\t' +
        str(results[test_name]['Pinned-Pinned']) + '\n'
    )
out.close()

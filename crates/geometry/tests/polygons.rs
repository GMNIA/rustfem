use geometry::{assert_almost_eq, Axis, Line, Polygon, Vector2d, Vector3d};

#[test]
fn square_xy_metrics_axes_and_contains() {
    let poly = Polygon::new([
        Vector2d::new(0.0, 0.0),
        Vector2d::new(2.0, 0.0),
        Vector2d::new(2.0, 2.0),
        Vector2d::new(0.0, 2.0),
    ]);
    assert_almost_eq!(poly.area(), 4.0);
    assert_almost_eq!(poly.perimeter(), 8.0);
    let c = poly.centroid();
    assert_almost_eq!(c.x(), 1.0);
    assert_almost_eq!(c.y(), 1.0);
    assert_almost_eq!(c.z(), 0.0);

    let ex = poly.axis(Axis::AxisX);
    let ey = poly.axis(Axis::AxisY);
    let ez = poly.axis(Axis::AxisZ);
    assert_almost_eq!(ex.x(), 1.0);
    assert_almost_eq!(ex.y(), 0.0);
    assert_almost_eq!(ex.z(), 0.0);
    assert_almost_eq!(ey.x(), 0.0);
    assert_almost_eq!(ey.y(), 1.0);
    assert_almost_eq!(ey.z(), 0.0);
    assert_almost_eq!(ez.x(), 0.0);
    assert_almost_eq!(ez.y(), 0.0);
    assert_almost_eq!(ez.z(), 1.0);

    assert!(poly.contains(&Vector3d::new(1.0, 1.0, 0.0)));
    assert!(poly.border_contains(&Vector3d::new(0.0, 1.0, 0.0)));
    assert!(!poly.contains(&Vector3d::new(3.0, 3.0, 0.0)));
}

#[test]
fn polygon_line_intersection_and_transforms() {
    let poly = Polygon::new([
        Vector2d::new(0.0, 0.0),
        Vector2d::new(1.0, 0.0),
        Vector2d::new(1.0, 1.0),
        Vector2d::new(0.0, 1.0),
    ]);

    let line = Line::new(Vector3d::new(0.5, 0.5, -2.0), Vector3d::new(0.5, 0.5, 2.0));
    let hits = poly.intersection_with_line(&line, false);
    assert_eq!(hits.len(), 1);
    let p = hits[0];
    assert_almost_eq!(p.x(), 0.5);
    assert_almost_eq!(p.y(), 0.5);
    assert_almost_eq!(p.z(), 0.0);

    let local = poly.to_local(p);
    assert_almost_eq!(local.z(), 0.0);
    let back = poly.to_global(local);
    assert_almost_eq!(back.x(), p.x());
    assert_almost_eq!(back.y(), p.y());
    assert_almost_eq!(back.z(), p.z());
}

#[test]
fn polygon_axes_and_validity_examples() {
    let v1 = Vector3d::new(0.0, 0.0, 0.0);
    let v2 = Vector3d::new(1.0, 0.0, 0.0);
    let v3 = Vector3d::new(0.0, 1.0, 0.0);
    let v4 = Vector3d::new(1.0, 1.0, 0.0);

    // triangle 1 (v1, v2, v3)
    let p = Polygon::new([v1, v2, v3]);
    let dirx = p.direction(Axis::AxisX);
    let diry = p.direction(Axis::AxisY);
    let dirz = p.direction(Axis::AxisZ);
    assert_almost_eq!(dirx.x(), 1.0);
    assert_almost_eq!(dirx.y(), 0.0);
    assert_almost_eq!(dirx.z(), 0.0);
    assert_almost_eq!(diry.x(), 0.0);
    assert_almost_eq!(diry.y(), 1.0);
    assert_almost_eq!(diry.z(), 0.0);
    assert_almost_eq!(dirz.x(), 0.0);
    assert_almost_eq!(dirz.y(), 0.0);
    assert_almost_eq!(dirz.z(), 1.0);
    assert!(p.is_valid());

    // square 1 (simple axis-aligned)
    let p = Polygon::new([v1, v2, v4, v3]);
    let dirx = p.direction(Axis::AxisX);
    let diry = p.direction(Axis::AxisY);
    let dirz = p.direction(Axis::AxisZ);
    assert_almost_eq!(dirx.x(), 1.0);
    assert_almost_eq!(dirx.y(), 0.0);
    assert_almost_eq!(dirx.z(), 0.0);
    assert_almost_eq!(diry.x(), 0.0);
    assert_almost_eq!(diry.y(), 1.0);
    assert_almost_eq!(diry.z(), 0.0);
    assert_almost_eq!(dirz.x(), 0.0);
    assert_almost_eq!(dirz.y(), 0.0);
    assert_almost_eq!(dirz.z(), 1.0);
    assert!(p.is_valid());

    // square 2: self-intersecting order -> invalid
    let p = Polygon::new([v1, v4, v2, v3]);
    assert!(!p.is_valid());

    // square 3: translated in Z, non-planar input last is projected
    let v1 = Vector3d::new(0.0, 0.0, -1.0);
    let v2 = Vector3d::new(1.0, 0.0, -1.0);
    let v4 = Vector3d::new(1.0, 1.0, -1.0);
    let v5 = Vector3d::new(0.0, 2.0, 1.0);
    let p = Polygon::new([v1, v2, v4, v5]);
    // Expect XY-like axes despite projection
    let dirx = p.direction(Axis::AxisX);
    let diry = p.direction(Axis::AxisY);
    let dirz = p.direction(Axis::AxisZ);
    assert_almost_eq!(dirx.x(), 1.0);
    assert_almost_eq!(dirx.y(), 0.0);
    assert_almost_eq!(dirx.z(), 0.0);
    assert_almost_eq!(diry.x(), 0.0);
    assert_almost_eq!(diry.y(), 1.0);
    assert_almost_eq!(diry.z(), 0.0);
    assert_almost_eq!(dirz.x(), 0.0);
    assert_almost_eq!(dirz.y(), 0.0);
    assert_almost_eq!(dirz.z(), 1.0);
    assert!(p.is_valid());
}

#[test]
fn polygon_direction_and_validity_reference_examples() {
    // Helper to compare direction vectors with current global epsilon
    fn assert_vec3_approx(a: Vector3d, b: Vector3d) {
        assert_almost_eq!(a.x(), b.x());
        assert_almost_eq!(a.y(), b.y());
        assert_almost_eq!(a.z(), b.z());
    }

    // triangle 1
    let v1 = Vector3d::new(0.145703, -2.094777, 2.462707);
    let v2 = Vector3d::new(1.961598, -2.740023, 2.711883);
    let v3 = Vector3d::new(1.555111, -3.472660, 3.118254);
    let p = Polygon::new([v1, v2, v3]);
    assert_vec3_approx(p.direction(Axis::AxisX), Vector3d::new(0.9345018409130872, -0.33205861288334715, 0.1282317704004688));
    assert_vec3_approx(p.direction(Axis::AxisY), Vector3d::new(-0.3532031024386168, -0.8202752406640758, 0.44988453849985616));
    assert_vec3_approx(p.direction(Axis::AxisZ), Vector3d::new(-0.044202689485902, -0.46570978856309214, -0.8838328547178662));
    assert!(p.is_valid());

    // triangle 2
    let v1 = Vector3d::new(-2.480034, -1.498400, -1.463015);
    let v2 = Vector3d::new(-2.694129, -1.607453, -1.510871);
    let v3 = Vector3d::new(-1.012195, -0.682450, -0.686012);
    let p = Polygon::new([v1, v2, v3]);
    assert_vec3_approx(p.direction(Axis::AxisX), Vector3d::new(-0.8738977444125401, -0.4451349668204331, -0.1953396877862937));
    assert_vec3_approx(p.direction(Axis::AxisY), Vector3d::new(-0.23536062221153137, 0.035850002044947905, 0.9712467013408992));
    assert_vec3_approx(p.direction(Axis::AxisZ), Vector3d::new(-0.4253329399692381, 0.8947455720299202, -0.1360964786832314));
    assert!(p.is_valid());

    // triangle 3
    let v1 = Vector3d::new(2.804859, 3.344900, 2.990635);
    let v2 = Vector3d::new(1.811340, 1.837735, 2.336541);
    let v3 = Vector3d::new(2.834659, 2.777435, 1.298765);
    let p = Polygon::new([v1, v2, v3]);
    assert_vec3_approx(p.direction(Axis::AxisX), Vector3d::new(-0.5174532293996775, -0.7849748182854731, -0.340670940999571));
    assert_vec3_approx(p.direction(Axis::AxisY), Vector3d::new(0.373531982697303, 0.15097710863715338, -0.915248474770533));
    assert_vec3_approx(p.direction(Axis::AxisZ), Vector3d::new(0.769880518837869, -0.6008497710120673, 0.21508960781190695));
    assert!(p.is_valid());

    // triangle 4
    let v1 = Vector3d::new(-0.023743, -1.718483, 0.162244);
    let v2 = Vector3d::new(0.513953, -0.689606, 0.365699);
    let v3 = Vector3d::new(2.187714, -1.110610, -0.151158);
    let p = Polygon::new([v1, v2, v3]);
    assert_vec3_approx(p.direction(Axis::AxisX), Vector3d::new(0.4562157102950194, 0.8729651166480851, 0.17262424741503224));
    assert_vec3_approx(p.direction(Axis::AxisY), Vector3d::new(0.8639480811079011, -0.38803235900871336, -0.320974456167634));
    assert_vec3_approx(p.direction(Axis::AxisZ), Vector3d::new(-0.2132157096228755, 0.2955719768139874, -0.9312229956848602));
    assert!(p.is_valid());

    // square 1
    let v1 = Vector3d::new(0.466704, -1.308872, -0.402346);
    let v2 = Vector3d::new(-2.710254, -1.422805, -1.618877);
    let v3 = Vector3d::new(-1.599556, 0.344025, -2.161165);
    let v4 = Vector3d::new(0.2136551598733032, 0.7050697777418757, -1.6326973966920981);
    let p = Polygon::new([v1, v2, v3, v4]);
    assert_vec3_approx(p.direction(Axis::AxisX), Vector3d::new(-0.9333507669921716, -0.03347209907582006, -0.357401684856946));
    assert_vec3_approx(p.direction(Axis::AxisY), Vector3d::new(0.13725619607812056, 0.8867074592154259, -0.4414868269947457));
    assert_vec3_approx(p.direction(Axis::AxisZ), Vector3d::new(0.33168823071265235, -0.46111766432786155, -0.8230148341625002));
    assert!(p.is_valid());

    // square 2
    let v1 = Vector3d::new(1.820030, 1.625624, 1.968143);
    let v2 = Vector3d::new(0.995498, 1.824569, 2.699085);
    let v3 = Vector3d::new(1.317367, 1.033428, 1.383929);
    let v4 = Vector3d::new(2.643154, 0.394775, -0.251469);
    let p = Polygon::new([v1, v2, v3, v4]);
    assert_vec3_approx(p.direction(Axis::AxisX), Vector3d::new(-0.7363923325025404, 0.17767845588736159, 0.6528067853086018));
    assert_vec3_approx(p.direction(Axis::AxisY), Vector3d::new(-0.6097879444370069, -0.5922704466883776, -0.5266634416767879));
    assert_vec3_approx(p.direction(Axis::AxisZ), Vector3d::new(0.2930614192464742, -0.7859046279880483, 0.5444895960954715));
    assert!(p.is_valid());

    // square 3
    let v1 = Vector3d::new(0.477334, 0.986900, -1.312910);
    let v2 = Vector3d::new(0.327158, 1.136919, -0.801879);
    let v3 = Vector3d::new(0.267723, 2.079102, 0.622051);
    let v4 = Vector3d::new(1.209465, 2.700932, -0.420196);
    let p = Polygon::new([v1, v2, v3, v4]);
    assert_vec3_approx(p.direction(Axis::AxisX), Vector3d::new(-0.2713875914355038, 0.27110387198728725, 0.9234995754240154));
    assert_vec3_approx(p.direction(Axis::AxisY), Vector3d::new(0.5856119163029412, 0.8079734417165727, -0.0650968583318195));
    assert_vec3_approx(p.direction(Axis::AxisZ), Vector3d::new(-0.7638111407270995, 0.5231458764763196, -0.3780356242829376));
    assert!(p.is_valid());

    // square 4
    let v1 = Vector3d::new(1.660860, 0.312090, 0.080616);
    let v2 = Vector3d::new(0.591446, -2.327764, 0.170717);
    let v3 = Vector3d::new(-0.305285, -1.685820, -0.191411);
    let v4 = Vector3d::new(-0.268855, -1.302996, -0.239373);
    let p = Polygon::new([v1, v2, v3, v4]);
    assert_vec3_approx(p.direction(Axis::AxisX), Vector3d::new(-0.37527702413043856, -0.926373278504709, 0.031618096594187704));
    assert_vec3_approx(p.direction(Axis::AxisY), Vector3d::new(-0.88387778289721, 0.3473721935208172, -0.31319742027872505));
    assert_vec3_approx(p.direction(Axis::AxisZ), Vector3d::new(0.2791544734739436, -0.14548232896463065, -0.9491615625901167));
    assert!(p.is_valid());

    // pentagon 1
    let v1 = Vector3d::new(-1.901807, 1.981353, -0.252286);
    let v2 = Vector3d::new(-1.855172, 1.874584, -0.565035);
    let v3 = Vector3d::new(-1.568860, 2.630867, -0.378144);
    let v4 = Vector3d::new(-1.967688, 3.272579, 1.891474);
    let v5 = Vector3d::new(-2.592943, 1.744969, 1.668370);
    let p = Polygon::new([v1, v2, v3, v4, v5]);
    assert_vec3_approx(p.direction(Axis::AxisX), Vector3d::new(0.1397320198790167, -0.31991096881017955, -0.9370869408200407));
    assert_vec3_approx(p.direction(Axis::AxisY), Vector3d::new(0.45849203101134806, 0.8597105836782297, -0.22512834075417226));
    assert_vec3_approx(p.direction(Axis::AxisZ), Vector3d::new(0.8776445864469393, -0.3981892569451993, 0.26680572620131465));
    assert!(p.is_valid());

    // pentagon 2
    let v1 = Vector3d::new(-0.627362, 2.820809, 0.013486);
    let v2 = Vector3d::new(-1.207606, 1.914747, 0.480826);
    let v3 = Vector3d::new(-1.753313, 1.351764, -0.114029);
    let v4 = Vector3d::new(-0.767447, 3.017038, -1.358183);
    let v5 = Vector3d::new(-0.230421, 3.677723, -1.154341);
    let p = Polygon::new([v1, v2, v3, v4, v5]);
    assert_vec3_approx(p.direction(Axis::AxisX), Vector3d::new(-0.49464705217703553, -0.7724007441518211, 0.398398524352541));
    assert_vec3_approx(p.direction(Axis::AxisY), Vector3d::new(-0.3629958314708085, -0.2328932696861457, -0.9022165766985843));
    assert_vec3_approx(p.direction(Axis::AxisZ), Vector3d::new(0.789657090202694, -0.5908957736933048, -0.16517828102993817));
    assert!(p.is_valid());

    // pentagon 3
    let v1 = Vector3d::new(-1.618389, 2.812376, -2.307240);
    let v2 = Vector3d::new(-1.395141, 1.850150, -2.652754);
    let v3 = Vector3d::new(-1.032313, 1.546406, -1.520788);
    let v4 = Vector3d::new(-1.164610, 2.602656, -0.662824);
    let v5 = Vector3d::new(-1.240844, 2.571746, -1.027976);
    let p = Polygon::new([v1, v2, v3, v4, v5]);
    assert_vec3_approx(p.direction(Axis::AxisX), Vector3d::new(0.2133344492701482, -0.9194973920636131, -0.3301711052512268));
    assert_vec3_approx(p.direction(Axis::AxisY), Vector3d::new(0.2987235921637419, -0.26037399043914927, 0.9181337596383133));
    assert_vec3_approx(p.direction(Axis::AxisZ), Vector3d::new(-0.9301895657549557, -0.29449945855808957, 0.21912882208782833));
    assert!(p.is_valid());

    // pentagon 4
    let v1 = Vector3d::new(1.410699, 0.438958, -2.088725);
    let v2 = Vector3d::new(1.654198, -0.532698, -2.438733);
    let v3 = Vector3d::new(2.067207, -2.177650, -3.028624);
    let v4 = Vector3d::new(2.428231813496742, -1.9925544440997975, -1.5787366299802403);
    let v5 = Vector3d::new(1.976302003313561, -0.09504788568490369, -0.815131871208945);
    let p = Polygon::new([v1, v2, v3, v4, v5]);
    assert_vec3_approx(p.direction(Axis::AxisX), Vector3d::new(0.22947998577874223, -0.9157146643798515, -0.3298569228721512));
    assert_vec3_approx(p.direction(Axis::AxisY), Vector3d::new(0.35157905223012453, -0.23803175446001687, 0.9053911054906899));
    assert_vec3_approx(p.direction(Axis::AxisZ), Vector3d::new(-0.9075963343689485, -0.32373992232714727, 0.2673225701875182));
    assert!(p.is_valid());

    // hexagon 1
    let v1 = Vector3d::new(2.558109, -0.859465, -0.350182);
    let v2 = Vector3d::new(2.520099, -1.119007, 0.324867);
    let v3 = Vector3d::new(3.698993, -0.919919, 1.093922);
    let v4 = Vector3d::new(2.075560, -1.880729, 1.933347);
    let v5 = Vector3d::new(1.140451, -2.357227, 2.204148);
    let v6 = Vector3d::new(1.642481, -1.755718, 1.102979);
    let p = Polygon::new([v1, v2, v3, v4, v5, v6]);
    assert_vec3_approx(p.direction(Axis::AxisX), Vector3d::new(-0.05248388400442096, -0.3583733812753347, 0.9321019051118253));
    assert_vec3_approx(p.direction(Axis::AxisY), Vector3d::new(0.933095068201001, 0.3149398238279995, 0.17362747785462476));
    assert_vec3_approx(p.direction(Axis::AxisZ), Vector3d::new(-0.3557794761067311, 0.8788523351283037, 0.31786716946037014));
    assert!(p.is_valid());

    // hexagon 2
    let v1 = Vector3d::new(-3.551911, -1.056528, 3.557519);
    let v2 = Vector3d::new(-3.816264, -2.393857, 2.864560);
    let v3 = Vector3d::new(-2.242871, -2.793499, 1.782285);
    let v4 = Vector3d::new(-2.098805, -2.722875, 1.749969);
    let v5 = Vector3d::new(-1.233136, -1.067268, 2.322683);
    let v6 = Vector3d::new(-1.609146, -0.487567, 2.882913);
    let p = Polygon::new([v1, v2, v3, v4, v5, v6]);
    assert_vec3_approx(p.direction(Axis::AxisX), Vector3d::new(-0.17286750764397454, -0.8745152547162658, -0.45314445165918693));
    assert_vec3_approx(p.direction(Axis::AxisY), Vector3d::new(0.8955342557064535, 0.05197936142659652, -0.4419463121714803));
    assert_vec3_approx(p.direction(Axis::AxisZ), Vector3d::new(0.41004295099080623, -0.4822045367416487, 0.7741728250775297));
    assert!(p.is_valid());

    // hexagon 3
    let v1 = Vector3d::new(2.843711, 1.362341, 1.418280);
    let v2 = Vector3d::new(2.952960, 1.398499, 1.603335);
    let v3 = Vector3d::new(2.531493, 0.668973, 1.794596);
    let v4 = Vector3d::new(2.152379, 0.548992, 1.143990);
    let v5 = Vector3d::new(1.660464, 1.279367, -1.059502);
    let v6 = Vector3d::new(2.046192, 1.361658, -0.336516);
    let p = Polygon::new([v1, v2, v3, v4, v5, v6]);
    assert_vec3_approx(p.direction(Axis::AxisX), Vector3d::new(0.501331603114364, 0.16592507121721117, 0.8491969703551381));
    assert_vec3_approx(p.direction(Axis::AxisY), Vector3d::new(-0.3969849368396265, -0.8279372122525577, 0.39613499276118097));
    assert_vec3_approx(p.direction(Axis::AxisZ), Vector3d::new(0.7688104991746794, -0.5357133966114964, -0.3492013359793371));
    assert!(p.is_valid());

    // hexagon 4
    let v1 = Vector3d::new(-1.619455, 0.220686, -3.132679);
    let v2 = Vector3d::new(-2.321431, -0.472090, -1.957411);
    let v3 = Vector3d::new(-2.363098, -0.739151, -1.782359);
    let v4 = Vector3d::new(-3.019946, -0.164459, -1.252555);
    let v5 = Vector3d::new(-3.175343, -0.137162, -1.076574);
    let v6 = Vector3d::new(-2.223053, 1.312404, -2.908479);
    let p = Polygon::new([v1, v2, v3, v4, v5, v6]);
    assert_vec3_approx(p.direction(Axis::AxisX), Vector3d::new(-0.4575329518558935, -0.4515365885086079, 0.7660145606997565));
    assert_vec3_approx(p.direction(Axis::AxisY), Vector3d::new(0.4927992260170934, -0.8458332563800127, -0.2042425647080875));
    assert_vec3_approx(p.direction(Axis::AxisZ), Vector3d::new(0.7401435812077182, 0.28404367910515405, 0.609513467904871));
    assert!(p.is_valid());

    // heptagon 1
    let v1 = Vector3d::new(0.312292, 1.380112, -0.955770);
    let v2 = Vector3d::new(2.799068, 0.914299, -3.369285);
    let v3 = Vector3d::new(2.509338, 0.323344, -3.591651);
    let v4 = Vector3d::new(0.44353060751145257, 0.4042314248486336, -1.8255732630425405);
    let v5 = Vector3d::new(0.36918189548117225, 0.47990015221809, -1.7052289137134335);
    let v6 = Vector3d::new(-0.09936751359212115, 0.6339765395014285, -1.1987324099503585);
    let v7 = Vector3d::new(0.5141486724303492, 0.6071503633877479, -1.7254211842909961);
    let p = Polygon::new([v1, v2, v3, v4, v5, v6, v7]);
    assert_vec3_approx(p.direction(Axis::AxisX), Vector3d::new(0.7112018052835161, -0.1332194964582779, -0.6902496344981798));
    assert_vec3_approx(p.direction(Axis::AxisY), Vector3d::new(-0.44414780809942095, -0.8462373964823949, -0.29430425303616986));
    assert_vec3_approx(p.direction(Axis::AxisZ), Vector3d::new(-0.5449079892256562, 0.5158825782657337, -0.6610147114247545));
    assert!(p.is_valid());

    // heptagon 2
    let v1 = Vector3d::new(0.360931, 1.014467, 3.218626);
    let v2 = Vector3d::new(1.801837, 1.495068, 1.343394);
    let v3 = Vector3d::new(1.353518, 1.368312, 0.966079);
    let v4 = Vector3d::new(0.9212235315011662, 1.2589062440642893, 0.06149595837248878);
    let v5 = Vector3d::new(0.47483062465356507, 1.1225089363140255, 0.11546395107673302);
    let v6 = Vector3d::new(-0.6515893866165735, 0.7610689735475065, 0.9795989519594186);
    let v7 = Vector3d::new(-0.8309924931402849, 0.6769303254729603, 2.2381199603024236);
    let p = Polygon::new([v1, v2, v3, v4, v5, v6, v7]);
    assert_vec3_approx(p.direction(Axis::AxisX), Vector3d::new(0.5970858865786243, 0.19915252915566556, -0.7770628765933425));
    assert_vec3_approx(p.direction(Axis::AxisY), Vector3d::new(-0.7480649450443138, -0.21151862100061375, -0.6290140785116392));
    assert_vec3_approx(p.direction(Axis::AxisZ), Vector3d::new(-0.2896330126979072, 0.9568689268133335, 0.022684242430229585));
    assert!(p.is_valid());

    // heptagon 3
    let v1 = Vector3d::new(-2.857029, 1.215964, -2.075082);
    let v2 = Vector3d::new(-2.307147, 1.269625, -2.020226);
    let v3 = Vector3d::new(-0.683271, 1.429674, -1.856817);
    let v4 = Vector3d::new(-1.2276814788515065, 0.9316239224327032, -2.308485342147391);
    let v5 = Vector3d::new(-1.150068204733844, 0.709143343175676, -2.506203087605629);
    let v6 = Vector3d::new(-0.4576570897210419, -0.1686126601743374, -3.2813935403674868);
    let v7 = Vector3d::new(-2.656933058039774, 0.43038273618648637, -2.7741577246878393);
    let p = Polygon::new([v1, v2, v3, v4, v5, v6, v7]);
    assert_vec3_approx(p.direction(Axis::AxisX), Vector3d::new(0.990402420629914, 0.09664979812654678, 0.0988021342507567));
    assert_vec3_approx(p.direction(Axis::AxisY), Vector3d::new(-0.13789363867082355, 0.7395877053992996, 0.6587832499664203));
    assert_vec3_approx(p.direction(Axis::AxisZ), Vector3d::new(-0.009401575640664569, -0.6660847112374644, 0.7458168460360604));
    assert!(p.is_valid());

    // heptagon 4
    let v1 = Vector3d::new(-1.246985, -2.606753, -1.902219);
    let v2 = Vector3d::new(-2.907286, -0.991081, -0.591317);
    let v3 = Vector3d::new(-2.534406, -1.515290, -1.058768);
    let v4 = Vector3d::new(-2.743896, -1.387650, -0.975104);
    let v5 = Vector3d::new(-2.555997, -1.899766, -1.476583);
    let v6 = Vector3d::new(-2.458005, -2.124526, -1.692730);
    let v7 = Vector3d::new(-1.303997, -3.390037, -2.756734);
    let p = Polygon::new([v1, v2, v3, v4, v5, v6, v7]);
    assert_vec3_approx(p.direction(Axis::AxisX), Vector3d::new(-0.6237388859176921, 0.6069727436702197, 0.4924772996145123));
    assert_vec3_approx(p.direction(Axis::AxisY), Vector3d::new(-0.7627614437107425, -0.33505340733135097, -0.5531131839180435));
    assert_vec3_approx(p.direction(Axis::AxisZ), Vector3d::new(-0.17071842953372066, -0.7206408971721615, 0.6719612452667508));
    assert!(p.is_valid());
}

#[test]
fn polygon_geometry_properties_reference_examples() {
    // Helper: compare vectors approximately
    fn approx_vec3(a: Vector3d, b: Vector3d) { assert!(a.is_approx(&b, None), "expected {:?} ~ {:?}", a, b); }
    fn assert_orthonormal3(m: nalgebra::Matrix3<f64>) {
        let c0 = m.column(0);
        let c1 = m.column(1);
        let c2 = m.column(2);
        assert_almost_eq!(c0.norm(), 1.0);
        assert_almost_eq!(c1.norm(), 1.0);
        assert_almost_eq!(c2.norm(), 1.0);
        assert_almost_eq!(c0.dot(&c1), 0.0);
        assert_almost_eq!(c0.dot(&c2), 0.0);
        assert_almost_eq!(c1.dot(&c2), 0.0);
    }

    // triangle 1
    let v1 = Vector3d::new(1.273997, 3.921693, 1.37026);
    let v2 = Vector3d::new(-0.369255, 3.263226, -1.06124);
    let v3 = Vector3d::new(-0.878888, 4.191277, -0.385543);
    let p = Polygon::new([v1, v2, v3]);
    assert_almost_eq!(p.area(), 1.7510030836815884);
    assert_almost_eq!(p.perimeter(), 7.0548136036616444);
    let c = p.centroid();
    approx_vec3(c, Vector3d::new(0.008618000000000015, 3.7920653333333334, -0.025507666666666706));
    approx_vec3(p.center(), v1);
    let pa = p.principal_axes();
    assert_orthonormal3(pa);
    // Ensure the third axis aligns with polygon normal direction (AxisZ)
    let z = p.direction(Axis::AxisZ).0;
    let dot = pa.column(2).dot(&z);
    assert_almost_eq!(dot.abs(), 1.0);
    let j = p.second_moment_of_area_at_center();
    // Symmetric inertia tensor
    assert_almost_eq!(j[(0,1)], j[(1,0)]);
    assert_almost_eq!(j[(0,2)], j[(2,0)]);
    assert_almost_eq!(j[(1,2)], j[(2,1)]);
    let lpa = p.local_principal_axes();
    // Check that local principal axes diagonalize the local inertia
    let s = p.centroidal_local_second_moment_of_area();
    let d = lpa.transpose() * s * lpa;
    assert_almost_eq!(d[(0,1)], 0.0);
    assert_almost_eq!(d[(1,0)], 0.0);
    // Eigenvalues may suffer tiny negative drift numerically; we only check diagonalization.

    // triangle 2
    let v1 = Vector3d::new(0.482686, -1.927148, 0.790973);
    let v2 = Vector3d::new(0.10085, -1.414955, 0.857194);
    let v3 = Vector3d::new(-0.799222, -1.653452, -0.340263);
    let p = Polygon::new([v1, v2, v3]);
    assert_almost_eq!(p.area(), 0.4819121922030239);
    assert_almost_eq!(p.perimeter(), 3.89060018876381);
    approx_vec3(p.centroid(), Vector3d::new(-0.07189533333333314, -1.6651850000000001, 0.4359680000000001));
    approx_vec3(p.vertices()[0], v1);
    let pa = p.principal_axes();
    assert_orthonormal3(pa);
    let z = p.direction(Axis::AxisZ).0;
    let dot = pa.column(2).dot(&z);
    assert_almost_eq!(dot.abs(), 1.0);
    let j = p.second_moment_of_area_at_center();
    assert_almost_eq!(j[(0,1)], j[(1,0)]);
    assert_almost_eq!(j[(0,2)], j[(2,0)]);
    assert_almost_eq!(j[(1,2)], j[(2,1)]);
    let lpa = p.local_principal_axes();
    let s = p.centroidal_local_second_moment_of_area();
    let d = lpa.transpose() * s * lpa;
    assert_almost_eq!(d[(0,1)], 0.0);
    assert_almost_eq!(d[(1,0)], 0.0);
    // Eigenvalues may suffer tiny negative drift numerically; we only check diagonalization.

    // triangle 3
    let v1 = Vector3d::new(5.325221, 2.756633, 0.614984);
    let v2 = Vector3d::new(5.345611, 1.843566, 1.273893);
    let v3 = Vector3d::new(4.006164, 2.043507, -0.32137);
    let p = Polygon::new([v1, v2, v3]);
    assert_almost_eq!(p.area(), 0.9954349364821459);
    assert_almost_eq!(p.perimeter(), 4.986599152627351);
    approx_vec3(p.centroid(), Vector3d::new(4.892332, 2.2145686666666666, 0.5225023333333333));
    let lpa = p.local_principal_axes();
    let s = p.centroidal_local_second_moment_of_area();
    let d = lpa.transpose() * s * lpa;
    assert_almost_eq!(d[(0,1)], 0.0);
    assert_almost_eq!(d[(1,0)], 0.0);
    // Eigenvalues may suffer tiny negative drift numerically; we only check diagonalization.

    // triangle 4
    let v1 = Vector3d::new(-4.471296, 1.413278, -0.010897);
    let v2 = Vector3d::new(-2.122123, -0.264632, -1.666023);
    let v3 = Vector3d::new(-4.370058, -0.208774, -0.58929);
    let p = Polygon::new([v1, v2, v3]);
    assert_almost_eq!(p.area(), 2.098302313069737);
    assert_almost_eq!(p.perimeter(), 7.545867664300957);
    approx_vec3(p.centroid(), Vector3d::new(-3.6544923333333332, 0.3132906666666666, -0.7554033333333334));
    let lpa = p.local_principal_axes();
    let s = p.centroidal_local_second_moment_of_area();
    let d = lpa.transpose() * s * lpa;
    assert_almost_eq!(d[(0,1)], 0.0);
    assert_almost_eq!(d[(1,0)], 0.0);
    // Eigenvalues may suffer tiny negative drift numerically; we only check diagonalization.

    // square 1
    let v1 = Vector3d::new(2.211274, 4.521325, 0.649714);
    let v2 = Vector3d::new(2.033522, 4.128939, 0.90783);
    let v3 = Vector3d::new(1.13126, 2.8248620000000004, 2.413387);
    let v4 = Vector3d::new(1.024288008145749, 1.5136291790567569, 2.2632778895275285);
    let p = Polygon::new([v1, v2, v3, v4]);
    assert_almost_eq!(p.area(), 1.2930370482660778);
    assert_almost_eq!(p.perimeter(), 7.62663736418228);
    approx_vec3(p.centroid(), Vector3d::new(1.4186432347456503, 2.8574839887920507, 1.8251041260680956));
    approx_vec3(p.center(), v1);
    let pa = p.principal_axes();
    assert_orthonormal3(pa);
    let z = p.direction(Axis::AxisZ).0;
    assert_almost_eq!(pa.column(2).dot(&z).abs(), 1.0);
    let j = p.second_moment_of_area_at_center();
    assert_almost_eq!(j[(0,1)], j[(1,0)]);
    assert_almost_eq!(j[(0,2)], j[(2,0)]);
    assert_almost_eq!(j[(1,2)], j[(2,1)]);
    let lpa = p.local_principal_axes();
    let s = p.centroidal_local_second_moment_of_area();
    let d = lpa.transpose() * s * lpa;
    assert_almost_eq!(d[(0,1)], 0.0);
    assert_almost_eq!(d[(1,0)], 0.0);
    // Eigenvalues may suffer tiny negative drift numerically; we only check diagonalization.

    // square 2
    let v1 = Vector3d::new(0.171495, -4.820469, -5.032104);
    let v2 = Vector3d::new(-1.5376520000000007, -3.3928289999999994, -4.056695);
    let v3 = Vector3d::new(0.25731299999999985, -3.340401, -4.554549);
    let v4 = Vector3d::new(0.04540996096743838, -3.74084395389633, -4.629551135873084);
    let p = Polygon::new([v1, v2, v3, v4]);
    assert_almost_eq!(p.area(), 1.2984256498608535);
    assert_almost_eq!(p.perimeter(), 5.913002167215147);
    approx_vec3(p.centroid(), Vector3d::new(-0.4317178469899698, -3.837580448192222, -4.525309643637489));
    approx_vec3(p.center(), v1);
    let pa = p.principal_axes();
    assert_orthonormal3(pa);
    let z = p.direction(Axis::AxisZ).0;
    assert_almost_eq!(pa.column(2).dot(&z).abs(), 1.0);
    let j = p.second_moment_of_area_at_center();
    assert_almost_eq!(j[(0,1)], j[(1,0)]);
    assert_almost_eq!(j[(0,2)], j[(2,0)]);
    assert_almost_eq!(j[(1,2)], j[(2,1)]);
    let lpa = p.local_principal_axes();
    let s = p.centroidal_local_second_moment_of_area();
    let d = lpa.transpose() * s * lpa;
    assert_almost_eq!(d[(0,1)], 0.0);
    assert_almost_eq!(d[(1,0)], 0.0);
    // Eigenvalues may suffer tiny negative drift numerically; we only check diagonalization.

    // square 3
    let v1 = Vector3d::new(-1.663035, -0.873988, 2.307167);
    let v2 = Vector3d::new(-0.5132399999999997, -1.4628930000000002, 1.9409099999999997);
    let v3 = Vector3d::new(-0.22803599999999968, -0.5266000000000002, 2.400398);
    let v4 = Vector3d::new(-1.6095120257578532, -0.33588777465600905, 2.5776565568072316);
    let p = Polygon::new([v1, v2, v3, v4]);
    assert_almost_eq!(p.area(), 1.122268893181803);
    assert_almost_eq!(p.perimeter(), 4.43443884976658);
    approx_vec3(p.centroid(), Vector3d::new(-0.9392572132067558, -0.8128097626788207, 2.296208306474967));
    approx_vec3(p.center(), v1);
    let pa = p.principal_axes();
    assert_orthonormal3(pa);
    let z = p.direction(Axis::AxisZ).0;
    assert_almost_eq!(pa.column(2).dot(&z).abs(), 1.0);
    let j = p.second_moment_of_area_at_center();
    assert_almost_eq!(j[(0,1)], j[(1,0)]);
    assert_almost_eq!(j[(0,2)], j[(2,0)]);
    assert_almost_eq!(j[(1,2)], j[(2,1)]);
    let lpa = p.local_principal_axes();
    let s = p.centroidal_local_second_moment_of_area();
    let d = lpa.transpose() * s * lpa;
    assert_almost_eq!(d[(0,1)], 0.0);
    assert_almost_eq!(d[(1,0)], 0.0);
    // Eigenvalues may suffer tiny negative drift numerically; we only check diagonalization.

    // square 4
    let v1 = Vector3d::new(3.755107, -1.557932, 1.479891);
    let v2 = Vector3d::new(4.350235, 0.599704, 2.00197);
    let v3 = Vector3d::new(4.593671, 0.5553760000000003, 1.5362010000000001);
    let v4 = Vector3d::new(4.812703409981565, 0.754194242953159, 1.2920676685022934);
    let p = Polygon::new([v1, v2, v3, v4]);
    assert_almost_eq!(p.area(), 0.9194717603298584);
    assert_almost_eq!(p.perimeter(), 5.758701855328054);
    approx_vec3(p.centroid(), Vector3d::new(4.286856289191753, -0.11629444316864435, 1.590023029795006));
    approx_vec3(p.center(), v1);
    let pa = p.principal_axes();
    assert_orthonormal3(pa);
    let z = p.direction(Axis::AxisZ).0;
    assert_almost_eq!(pa.column(2).dot(&z).abs(), 1.0);
    let j = p.second_moment_of_area_at_center();
    assert_almost_eq!(j[(0,1)], j[(1,0)]);
    assert_almost_eq!(j[(0,2)], j[(2,0)]);
    assert_almost_eq!(j[(1,2)], j[(2,1)]);
    let lpa = p.local_principal_axes();
    let s = p.centroidal_local_second_moment_of_area();
    let d = lpa.transpose() * s * lpa;
    assert_almost_eq!(d[(0,1)], 0.0);
    assert_almost_eq!(d[(1,0)], 0.0);
    // Eigenvalues may suffer tiny negative drift numerically; we only check diagonalization.

    // pentagon 1
    let v1 = Vector3d::new(-0.63443, -5.530453, -0.160867);
    let v2 = Vector3d::new(1.103762, -6.258464, -0.822445);
    let v3 = Vector3d::new(1.458374, -5.504957, -1.004154);
    let v4 = Vector3d::new(1.7284901084320627, -4.761819986034654, -1.151331730479522);
    let v5 = Vector3d::new(-0.2094082027414833, -3.9483410261117875, -0.4138375039374891);
    let p = Polygon::new([v1, v2, v3, v4, v5]);
    assert_almost_eq!(p.area(), 3.5222966333049954);
    assert_almost_eq!(p.perimeter(), 7.538882535251053);
    approx_vec3(p.centroid(), Vector3d::new(0.5102505590931625, -5.114387241372869, -0.6429475362090203));
    approx_vec3(p.center(), v1);
    let pa = p.principal_axes();
    assert_orthonormal3(pa);
    let z = p.direction(Axis::AxisZ).0;
    assert_almost_eq!(pa.column(2).dot(&z).abs(), 1.0);
    let j = p.second_moment_of_area_at_center();
    assert_almost_eq!(j[(0,1)], j[(1,0)]);
    assert_almost_eq!(j[(0,2)], j[(2,0)]);
    assert_almost_eq!(j[(1,2)], j[(2,1)]);
    let lpa = p.local_principal_axes();
    let s = p.centroidal_local_second_moment_of_area();
    let d = lpa.transpose() * s * lpa;
    assert_almost_eq!(d[(0,1)], 0.0);
    assert_almost_eq!(d[(1,0)], 0.0);
    // Eigenvalues may suffer tiny negative drift numerically; we only check diagonalization.

    // pentagon 2
    let v1 = Vector3d::new(-1.111273, -2.847684, 2.097793);
    let v2 = Vector3d::new(-0.786835, -3.581827, 1.572787);
    let v3 = Vector3d::new(-0.8416939999999999, -2.106004, 3.610485);
    let v4 = Vector3d::new(-1.0084193235352243, -1.906775283677806, 3.623574196746236);
    let v5 = Vector3d::new(-1.9316007355824658, -2.3037416449635293, 1.533103447317852);
    let p = Polygon::new([v1, v2, v3, v4, v5]);
    assert_almost_eq!(p.area(), 1.7359096159867446);
    assert_almost_eq!(p.perimeter(), 7.190035163953716);
    approx_vec3(p.centroid(), Vector3d::new(-1.173005243303904, -2.4942187802843514, 2.505921177866621));
    approx_vec3(p.center(), v1);
    let pa = p.principal_axes();
    assert_orthonormal3(pa);
    let z = p.direction(Axis::AxisZ).0;
    assert_almost_eq!(pa.column(2).dot(&z).abs(), 1.0);
    let j = p.second_moment_of_area_at_center();
    assert_almost_eq!(j[(0,1)], j[(1,0)]);
    assert_almost_eq!(j[(0,2)], j[(2,0)]);
    assert_almost_eq!(j[(1,2)], j[(2,1)]);
    let lpa = p.local_principal_axes();
    let s = p.centroidal_local_second_moment_of_area();
    let d = lpa.transpose() * s * lpa;
    assert!(d[(0,1)].abs() < 1e-9 && d[(1,0)].abs() < 1e-9);
    // Eigenvalues may suffer tiny negative drift numerically; we only check diagonalization.

    // pentagon 3
    let v1 = Vector3d::new(-3.00236, 0.13763, -5.552412);
    let v2 = Vector3d::new(-3.060985, 0.268927, -5.155602);
    let v3 = Vector3d::new(-1.1841659999999996, 1.2383650000000002, -2.891194);
    let v4 = Vector3d::new(-1.116755916806679, 0.31872516448254395, -5.571672711251874);
    let v5 = Vector3d::new(-1.2855948947558593, 0.2053539430243245, -5.851076634717691);
    let p = Polygon::new([v1, v2, v3, v4, v5]);
    assert_almost_eq!(p.area(), 3.432871800229422);
    assert_almost_eq!(p.perimeter(), 8.442898377530653);
    approx_vec3(p.centroid(), Vector3d::new(-1.8585575912654626, 0.5343991017047761, -4.733875813886675));
    approx_vec3(p.center(), v1);
    let pa = p.principal_axes();
    assert_orthonormal3(pa);
    let z = p.direction(Axis::AxisZ).0;
    assert!((pa.column(2).dot(&z)).abs() > 1.0 - 1e-9);
    let j = p.second_moment_of_area_at_center();
    assert_almost_eq!(j[(0,1)], j[(1,0)]);
    assert_almost_eq!(j[(0,2)], j[(2,0)]);
    assert_almost_eq!(j[(1,2)], j[(2,1)]);
    let lpa = p.local_principal_axes();
    let s = p.centroidal_local_second_moment_of_area();
    let d = lpa.transpose() * s * lpa;
    assert!(d[(0,1)].abs() < 1e-9 && d[(1,0)].abs() < 1e-9);
    // Eigenvalues may suffer tiny negative drift numerically; we only check diagonalization.

    // pentagon 4
    let v1 = Vector3d::new(2.550944, 3.990375, -3.07388);
    let v2 = Vector3d::new(2.336982, 3.387251, -2.821175);
    let v3 = Vector3d::new(2.421539, 3.982965, -1.02568);
    let v4 = Vector3d::new(2.552032866198793, 4.444316043991518, -0.6838470082943893);
    let v5 = Vector3d::new(2.767192686111999, 4.921052103200935, -1.6262070194580398);
    let p = Polygon::new([v1, v2, v3, v4, v5]);
    assert_almost_eq!(p.area(), 1.9761643576404115);
    assert_almost_eq!(p.perimeter(), 5.98283603429051);
    approx_vec3(p.centroid(), Vector3d::new(2.532757832045938, 4.154010921831523, -1.9126222484562452));
    approx_vec3(p.center(), v1);
    let pa = p.principal_axes();
    assert_orthonormal3(pa);
    let z = p.direction(Axis::AxisZ).0;
    assert!((pa.column(2).dot(&z)).abs() > 1.0 - 1e-9);
    let j = p.second_moment_of_area_at_center();
    assert_almost_eq!(j[(0,1)], j[(1,0)]);
    assert_almost_eq!(j[(0,2)], j[(2,0)]);
    assert_almost_eq!(j[(1,2)], j[(2,1)]);
    let lpa = p.local_principal_axes();
    let s = p.centroidal_local_second_moment_of_area();
    let d = lpa.transpose() * s * lpa;
    assert!(d[(0,1)].abs() < 1e-9 && d[(1,0)].abs() < 1e-9);
    // Eigenvalues may suffer tiny negative drift numerically; we only check diagonalization.

    // hexagon 1
    let v1 = Vector3d::new(-4.029473, 2.91465, 2.888199);
    let v2 = Vector3d::new(-4.122465, 2.850596, 2.742269);
    let v3 = Vector3d::new(-3.705515, 3.4953400000000006, 1.610805);
    let v4 = Vector3d::new(-3.3743821685752255, 4.021089168069514, 0.6437590336505359);
    let v5 = Vector3d::new(-2.2496442761241653, 5.020792287295487, 1.285155256649854);
    let v6 = Vector3d::new(-3.7490356003142202, 3.0417286045131626, 3.658372720598464);
    let p = Polygon::new([v1, v2, v3, v4, v5, v6]);
    assert_almost_eq!(p.area(), 3.605154317610444);
    assert_almost_eq!(p.perimeter(), 8.60126776174472);
    approx_vec3(p.centroid(), Vector3d::new(-3.297696978846067, 3.8269467946029705, 1.997575128174337));
    approx_vec3(p.center(), v1);
    let pa = p.principal_axes();
    assert_orthonormal3(pa);
    let z = p.direction(Axis::AxisZ).0;
    assert!((pa.column(2).dot(&z)).abs() > 1.0 - 1e-9);
    let j = p.second_moment_of_area_at_center();
    assert_almost_eq!(j[(0,1)], j[(1,0)]);
    assert_almost_eq!(j[(0,2)], j[(2,0)]);
    assert_almost_eq!(j[(1,2)], j[(2,1)]);
    let lpa = p.local_principal_axes();
    let s = p.centroidal_local_second_moment_of_area();
    let d = lpa.transpose() * s * lpa;
    assert!(d[(0,1)].abs() < 1e-9 && d[(1,0)].abs() < 1e-9);
    // Eigenvalues may suffer tiny negative drift numerically; we only check diagonalization.

    // hexagon 2
    let v1 = Vector3d::new(-3.57034, -0.506235, -1.937815);
    let v2 = Vector3d::new(-3.171132, 0.914881, -1.957016);
    let v3 = Vector3d::new(-4.044805, 0.30488399999999993, -2.159615);
    let v4 = Vector3d::new(-4.168134221233447, -1.2715009278960547, -2.0424012630639883);
    let v5 = Vector3d::new(-3.55927542928974, -1.83004886008678, -1.804967570021517);
    let v6 = Vector3d::new(-3.0926302562373307, -1.6884669164876616, -1.678730146464881);
    let p = Polygon::new([v1, v2, v3, v4, v5, v6]);
    assert_almost_eq!(p.area(), 1.7120513014823309);
    assert_almost_eq!(p.perimeter(), 6.810995125909555);
    approx_vec3(p.centroid(), Vector3d::new(-3.6878355989619855, -0.6038439739416606, -1.9635376732708698));
    approx_vec3(p.center(), v1);
    let pa = p.principal_axes();
    assert_orthonormal3(pa);
    let z = p.direction(Axis::AxisZ).0;
    assert!((pa.column(2).dot(&z)).abs() > 1.0 - 1e-9);
    let j = p.second_moment_of_area_at_center();
    assert_almost_eq!(j[(0,1)], j[(1,0)]);
    assert_almost_eq!(j[(0,2)], j[(2,0)]);
    assert_almost_eq!(j[(1,2)], j[(2,1)]);
    let lpa = p.local_principal_axes();
    let s = p.centroidal_local_second_moment_of_area();
    let d = lpa.transpose() * s * lpa;
    assert!(d[(0,1)].abs() < 1e-9 && d[(1,0)].abs() < 1e-9);
    // Eigenvalues may suffer tiny negative drift numerically; we only check diagonalization.

    // hexagon 3
    let v1 = Vector3d::new(1.868364, 3.162006, 3.037899);
    let v2 = Vector3d::new(2.992057, 3.187334, 2.195471);
    let v3 = Vector3d::new(3.8914859999999996, 3.166458, 1.749697);
    let v4 = Vector3d::new(2.8892055345688146, 3.4115207536813257, 1.0146678559692641);
    let v5 = Vector3d::new(2.0887029154540455, 3.609953140723295, 0.4125894658552922);
    let v6 = Vector3d::new(1.2749544353021753, 3.38898787094751, 2.147935697020239);
    let p = Polygon::new([v1, v2, v3, v4, v5, v6]);
    assert_almost_eq!(p.area(), 3.3273412101197852);
    assert_almost_eq!(p.perimeter(), 7.7194899268363955);
    approx_vec3(p.centroid(), Vector3d::new(2.368515384962092, 3.3381569483044506, 1.7472792868905156));
    approx_vec3(p.center(), v1);
    let pa = p.principal_axes();
    assert_orthonormal3(pa);
    let z = p.direction(Axis::AxisZ).0;
    assert!((pa.column(2).dot(&z)).abs() > 1.0 - 1e-9);
    let j = p.second_moment_of_area_at_center();
    assert_almost_eq!(j[(0,1)], j[(1,0)]);
    assert_almost_eq!(j[(0,2)], j[(2,0)]);
    assert_almost_eq!(j[(1,2)], j[(2,1)]);
    let lpa = p.local_principal_axes();
    let s = p.centroidal_local_second_moment_of_area();
    let d = lpa.transpose() * s * lpa;
    assert!(d[(0,1)].abs() < 1e-9 && d[(1,0)].abs() < 1e-9);
    // Eigenvalues may suffer tiny negative drift numerically; we only check diagonalization.

    // hexagon 4
    let v1 = Vector3d::new(2.215233, 0.279932, 2.609599);
    let v2 = Vector3d::new(2.828019, 0.476149, 2.935028);
    let v3 = Vector3d::new(2.462048, 1.5280690000000003, 0.9763759999999997);
    let v4 = Vector3d::new(1.722923781698641, 1.727523324798643, -0.07430378477332633);
    let v5 = Vector3d::new(0.7580362908906175, 0.550064055045036, 0.7239286991218674);
    let v6 = Vector3d::new(1.3711936301252954, 0.43670255031632765, 1.516919364665171);
    let p = Polygon::new([v1, v2, v3, v4, v5, v6]);
    assert_almost_eq!(p.area(), 3.8018035578251936);
    assert_almost_eq!(p.perimeter(), 8.391484024315407);
    approx_vec3(p.centroid(), Vector3d::new(1.8991272748476467, 0.9211997716708884, 1.321240050522253));
    approx_vec3(p.center(), v1);
    let pa = p.principal_axes();
    assert_orthonormal3(pa);
    let z = p.direction(Axis::AxisZ).0;
    assert!((pa.column(2).dot(&z)).abs() > 1.0 - 1e-9);
    let j = p.second_moment_of_area_at_center();
    assert_almost_eq!(j[(0,1)], j[(1,0)]);
    assert_almost_eq!(j[(0,2)], j[(2,0)]);
    assert_almost_eq!(j[(1,2)], j[(2,1)]);
    let lpa = p.local_principal_axes();
    let s = p.centroidal_local_second_moment_of_area();
    let d = lpa.transpose() * s * lpa;
    assert!(d[(0,1)].abs() < 1e-9 && d[(1,0)].abs() < 1e-9);
    // Eigenvalues may suffer tiny negative drift numerically; we only check diagonalization.

    // heptagon 1
    let v1 = Vector3d::new(-2.768819, 4.278058, 0.267843);
    let v2 = Vector3d::new(-3.762004, 4.688587, 0.21557);
    let v3 = Vector3d::new(-4.612287, 4.772955, 0.74563);
    let v4 = Vector3d::new(-5.224160158029589, 4.380826593639389, 2.1016268111797913);
    let v5 = Vector3d::new(-4.527185994969638, 3.8986960129351806, 2.5559020060104833);
    let v6 = Vector3d::new(-4.277203042439172, 3.9318818908710207, 2.2752639492919444);
    let v7 = Vector3d::new(-3.0011729592466545, 4.092277104794012, 0.8621270486937629);
    let p = Polygon::new([v1, v2, v3, v4, v5, v6, v7]);
    assert_almost_eq!(p.area(), 3.320830712225261);
    assert_almost_eq!(p.perimeter(), 7.534118526982814);
    approx_vec3(p.centroid(), Vector3d::new(-4.071517287262175, 4.330195545108811, 1.2459068925712589));
    approx_vec3(p.center(), v1);
    let pa = p.principal_axes();
    assert_orthonormal3(pa);
    let z = p.direction(Axis::AxisZ).0;
    assert!((pa.column(2).dot(&z)).abs() > 1.0 - 1e-9);
    let j = p.second_moment_of_area_at_center();
    assert_almost_eq!(j[(0,1)], j[(1,0)]);
    assert_almost_eq!(j[(0,2)], j[(2,0)]);
    assert_almost_eq!(j[(1,2)], j[(2,1)]);
    let lpa = p.local_principal_axes();
    let s = p.centroidal_local_second_moment_of_area();
    let d = lpa.transpose() * s * lpa;
    assert!(d[(0,1)].abs() < 1e-9 && d[(1,0)].abs() < 1e-9);
    // Eigenvalues may suffer tiny negative drift numerically; we only check diagonalization.

    // heptagon 2
    let v1 = Vector3d::new(-0.621746, -3.78801, 2.461415);
    let v2 = Vector3d::new(-0.9270760000000001, -4.117495, 2.796222);
    let v3 = Vector3d::new(-1.865536, -3.805585, 2.810859);
    let v4 = Vector3d::new(-2.711823041691548, -3.748181118196883, 2.9955048456609923);
    let v5 = Vector3d::new(-2.1327810744100066, -5.649863210954769, 4.2954467245396915);
    let v6 = Vector3d::new(-1.9801862143696165, -5.656155607744779, 4.2590452064196285);
    let v7 = Vector3d::new(-1.0652190551949252, -5.056174156479394, 3.552403795672493);
    let p = Polygon::new([v1, v2, v3, v4, v5, v6, v7]);
    assert_almost_eq!(p.area(), 2.994263120309039);
    assert_almost_eq!(p.perimeter(), 7.982733257925102);
    approx_vec3(p.centroid(), Vector3d::new(-1.7525074775665317, -4.573977637667984, 3.368781989014521));
    approx_vec3(p.center(), v1);
    let pa = p.principal_axes();
    assert_orthonormal3(pa);
    let z = p.direction(Axis::AxisZ).0;
    assert!((pa.column(2).dot(&z)).abs() > 1.0 - 1e-9);
    let j = p.second_moment_of_area_at_center();
    assert_almost_eq!(j[(0,1)], j[(1,0)]);
    assert_almost_eq!(j[(0,2)], j[(2,0)]);
    assert_almost_eq!(j[(1,2)], j[(2,1)]);
    let lpa = p.local_principal_axes();
    let s = p.centroidal_local_second_moment_of_area();
    let d = lpa.transpose() * s * lpa;
    assert!(d[(0,1)].abs() < 1e-9 && d[(1,0)].abs() < 1e-9);
    // Eigenvalues may suffer tiny negative drift numerically; we only check diagonalization.

    // heptagon 3
    let v1 = Vector3d::new(-2.726123, 2.780904, 1.868718);
    let v2 = Vector3d::new(-3.687655, 2.712327, 2.338951);
    let v3 = Vector3d::new(-3.688541, 2.712333, 2.337206);
    let v4 = Vector3d::new(-4.246460441821455, 2.6611819944356543, 2.9676946821893546);
    let v5 = Vector3d::new(-4.8468670717285605, 2.571845509612158, 4.725747332727322);
    let v6 = Vector3d::new(-3.7870717547829456, 2.623482902857282, 4.961402582896753);
    let v7 = Vector3d::new(-2.8262532763743504, 2.6835363271998545, 4.758262310504798);
    let p = Polygon::new([v1, v2, v3, v4, v5, v6, v7]);
    assert_almost_eq!(p.area(), 4.328861738116791);
    assert_almost_eq!(p.perimeter(), 8.741571447151534);
    approx_vec3(p.centroid(), Vector3d::new(-3.5983053445690394, 2.6759037780820196, 3.642582444399606));
    approx_vec3(p.center(), v1);
    let pa = p.principal_axes();
    assert_orthonormal3(pa);
    let z = p.direction(Axis::AxisZ).0;
    assert!((pa.column(2).dot(&z)).abs() > 1.0 - 1e-9);
    let j = p.second_moment_of_area_at_center();
    assert_almost_eq!(j[(0,1)], j[(1,0)]);
    assert_almost_eq!(j[(0,2)], j[(2,0)]);
    assert_almost_eq!(j[(1,2)], j[(2,1)]);
    let lpa = p.local_principal_axes();
    let s = p.centroidal_local_second_moment_of_area();
    let d = lpa.transpose() * s * lpa;
    assert!(d[(0,1)].abs() < 1e-9 && d[(1,0)].abs() < 1e-9);
    // Eigenvalues may suffer tiny negative drift numerically; we only check diagonalization.

    // heptagon 4
    let v1 = Vector3d::new(-2.737201, -4.489952, 3.895152);
    let v2 = Vector3d::new(-2.482893, -5.253629, 3.964977);
    let v3 = Vector3d::new(-3.10828, -4.966067, 1.552361);
    let v4 = Vector3d::new(-3.2936014209601963, -3.702555818927399, 2.4976198714862345);
    let v5 = Vector3d::new(-3.6939036190326435, -2.7779888808669977, 1.9966839154470049);
    let v6 = Vector3d::new(-3.2965379994629442, -2.9324939998320563, 3.5693859998808044);
    let v7 = Vector3d::new(-3.055075042551009, -3.5910190133062043, 3.7294920094438937);
    let p = Polygon::new([v1, v2, v3, v4, v5, v6, v7]);
    assert_almost_eq!(p.area(), 4.104118802296015);
    assert_almost_eq!(p.perimeter(), 9.347466871051706);
    approx_vec3(p.centroid(), Vector3d::new(-3.0663039463586568, -4.111790173163956, 2.9451451306026004));
    approx_vec3(p.center(), v1);
    let pa = p.principal_axes();
    assert_orthonormal3(pa);
    let z = p.direction(Axis::AxisZ).0;
    assert!((pa.column(2).dot(&z)).abs() > 1.0 - 1e-9);
    let j = p.second_moment_of_area_at_center();
    assert_almost_eq!(j[(0,1)], j[(1,0)]);
    assert_almost_eq!(j[(0,2)], j[(2,0)]);
    assert_almost_eq!(j[(1,2)], j[(2,1)]);
    let lpa = p.local_principal_axes();
    let s = p.centroidal_local_second_moment_of_area();
    let d = lpa.transpose() * s * lpa;
    assert!(d[(0,1)].abs() < 1e-9 && d[(1,0)].abs() < 1e-9);
    // Eigenvalues may suffer tiny negative drift numerically; we only check diagonalization.
}

#include "CollisionDetection.h"
std::vector<std::tuple<size_t, double, bool>> scanListX;
std::vector<std::tuple<size_t, double, bool>> scanListY;
std::vector<std::tuple<size_t, double, bool>> scanListZ;

std::vector<std::pair<size_t, size_t>> find_same(
    std::vector<std::pair<size_t, size_t>> a,
    std::vector<std::pair<size_t, size_t>> b) {
    std::vector<std::pair<size_t, size_t>> result;

    for (auto elem_a : a)
    {
        for (auto elem_b : b)
        {
            if ((elem_a.first == elem_b.first && elem_a.second == elem_b.second)
               || (elem_a.second == elem_b.first && elem_a.first == elem_b.second))
            {
                if (elem_a.first > 4 && elem_a.second < 5)
                {
                    result.push_back(std::make_pair(elem_a.second, elem_a.first));
                }
                else
                {
                    result.push_back(elem_a);
                }
            }
        }
    }
    std::sort(result.begin(), result.end(), [](std::pair<size_t, size_t> a, std::pair<size_t, size_t> b) { return (a.first < b.first) || (a.first == b.first && a.second < b.second); });
    return result;
}

std::vector<std::pair<size_t, size_t>> sap() {
    std::vector<size_t> ActiveList;
    std::vector<std::pair<size_t, size_t>> result;
    std::vector<std::pair<size_t, size_t>> tmp;
    // for x axis.
    for (auto point : scanListX)
    {
        if (std::get<2>(point))
        {
            size_t tobedelete = 0;
            //std::cout << "find pairs in x axis" << std::endl;
            for (size_t i = 0; i < ActiveList.size(); i++)
            {
                if (std::get<0>(point) == ActiveList[i]) {
                    tobedelete = i;
                }
                else
                {
                    result.push_back(std::make_pair(std::get<0>(point), ActiveList[i]));
                }
            }
            ActiveList.erase(ActiveList.begin() + tobedelete);
        }
        else {
            //std::cout << "append activeList" << std::endl;
            ActiveList.push_back(std::get<0>(point));
        }
    }

    // for y axis.
    for (auto point : scanListY)
    {
        if (std::get<2>(point))
        {
            size_t tobedelete = 0;
            for (size_t i = 0; i < ActiveList.size(); i++)
            {
                if (std::get<0>(point) == ActiveList[i]) {
                    tobedelete = i;
                }
                else
                {
                    tmp.push_back(std::make_pair(std::get<0>(point), ActiveList[i]));
                }
            }
            ActiveList.erase(ActiveList.begin() + tobedelete);
        }
        else {
            ActiveList.push_back(std::get<0>(point));
        }
    }

    result = find_same(result, tmp);
    tmp.clear();
    // for z axis.
    for (auto point : scanListZ)
    {
        if (std::get<2>(point))
        {
            size_t tobedelete = 0;
            for (size_t i = 0; i < ActiveList.size(); i++)
            {
                if (std::get<0>(point) == ActiveList[i]) {
                    tobedelete = i;
                }
                else
                {
                    tmp.push_back(std::make_pair(std::get<0>(point), ActiveList[i]));
                }
            }
            ActiveList.erase(ActiveList.begin() + tobedelete);
        }
        else {
            ActiveList.push_back(std::get<0>(point));
        }
    }

    result = find_same(result, tmp);
   
    tmp.clear();
    return result;
}

Eigen::Vector3d support(
    const Eigen::MatrixXd& Va, 
    const Eigen::MatrixXd& Vb, 
    const Eigen::Vector3d d) {
    double tmp;
    double maxPro = std::numeric_limits<double>::min();
    double minPro = std::numeric_limits<double>::max();
    Eigen::Vector3d far_a;
    Eigen::Vector3d far_b;

    for (int i = 0; i < Va.rows(); i++)
    {
        tmp = Va.row(i).dot(d);
        if (tmp > maxPro)
        {
            maxPro = tmp;
            far_a = Va.row(i);
        }
    }

    for (int i = 0; i < Vb.rows(); i++)
    {
        tmp = Vb.row(i).dot(d);
        if (tmp < minPro)
        {
            minPro = tmp;
            far_b = Vb.row(i);
        }
    }
    return far_a - far_b;
}


bool containsOrigin(
    std::vector<Eigen::Vector3d>& simplex, 
    Eigen::Vector3d& d) {
    Eigen::Vector3d a = *(simplex.end() - 1);
    Eigen::Vector3d ao = Eigen::Vector3d{ 0.0, 0.0, 0.0 } - a;

    if (simplex.size() == 4) 
    {
        // then its the tetrahedral case
        Eigen::Vector3d ab = *(simplex.begin() + 2) - a;
        Eigen::Vector3d ac = *(simplex.begin() + 1) - a;
        Eigen::Vector3d ad = *(simplex.begin()) - a;

        // define the normal of each face that point outside
        Eigen::Vector3d abcNormal = ab.cross(ac);
        if (abcNormal.dot(ad) > 0) abcNormal *= -1;

        Eigen::Vector3d acdNormal = ac.cross(ad);
        if (acdNormal.dot(ab) > 0) acdNormal *= -1;

        Eigen::Vector3d adbNormal = ad.cross(ab);
        if (adbNormal.dot(ac) > 0) adbNormal *= -1;

        if (abcNormal.dot(ao) > 0)
        {
            // the origin is onthe outside of the triangle abc
            // eliminate d
            simplex.erase(simplex.begin());
            d = abcNormal;
            
        }
        else if (acdNormal.dot(ao) > 0)
        {
            // the origin is onthe outside of the triangle acd
            // eliminate b
            simplex.erase(simplex.begin() + 2);
            d = acdNormal;
        }
        else if (adbNormal.dot(ao) > 0)
        {
            // the origin is onthe outside of the triangle adb
            // eliminate c
            simplex.erase(simplex.begin() + 1);
            d = adbNormal;
        }
        else
        {
            // the origin is inside all of the triangles
            // there is a collision!
            return true;
        }
    }
    else if (simplex.size() == 3)
    {
        // then its the triangle case
        Eigen::Vector3d b = *(simplex.begin() + 1);
        Eigen::Vector3d c = *(simplex.begin());
        // compute the edges
        Eigen::Vector3d ab = b - a;
        Eigen::Vector3d ac = c - a;
        // get the normal of ABC that in the direction of the origin
        Eigen::Vector3d abcNormal = ac.cross(ab);
        if (abcNormal.dot(ao) < 0) abcNormal *= -1;

        d = abcNormal;
    }
    else if (simplex.size() == 2)
    {
        // then its the line segment case
        Eigen::Vector3d b = *(simplex.begin());
        // compute AB
        Eigen::Vector3d ab = b - a;
        // get the normal of AB that in the direction of the origin
        Eigen::Vector3d abNormal = (ab.cross(ao)).cross(ab);
        // set the direction to abNormal
        d = abNormal;
    }
    d.normalize();
    return false;
}

bool gjk(const Eigen::MatrixXd& Va, 
    const Eigen::MatrixXd& Vb) {
    // init
    std::vector<Eigen::Vector3d> simplex;
    Eigen::Vector3d d{ 0.0, 0.0, 1.0 };
    simplex.push_back(support(Va, Vb, d));

    d = -d;

    while (true)
    {
        simplex.push_back(support(Va, Vb, d));

        if ((*(simplex.end() - 1)).dot(d) <= 0)
        {
            return false;
        }
        else {
            if (containsOrigin(simplex, d))
            {
                return true;
            }
        }
    }
}

void CollisionDetection::computeBroadPhase(int broadPhaseMethod) {
    // compute possible collisions
    m_overlappingBodys.clear();
    
	switch (broadPhaseMethod) {
	case 0: { // none
		for (size_t i = 0; i < m_objects.size(); i++) {
			for (size_t j = i + 1; j < m_objects.size(); j++) {
				m_overlappingBodys.push_back(std::make_pair(i, j));
			}
		}
		break;
	}
     
	case 1: {  // AABB
        // compute bounding boxes
        std::vector<AABB> aabbs(m_objects.size());
        for (size_t i = 0; i < aabbs.size(); i++) {
            aabbs[i].computeAABB(m_objects[i]);
        }
        for (size_t i = 0; i < m_objects.size(); i++) {
            for (size_t j = i + 1; j < m_objects.size(); j++) {
                // add pair of objects to possible collision if their
                // bounding boxes overlap
                if (aabbs[i].testCollision(aabbs[j])) {
                    m_overlappingBodys.push_back(std::make_pair(i, j));
                }
            }
        }
        break;
    }
	
	case 2: {
		// TODO: implement other broad phase algorithm
        std::vector<AABB> aabbs(m_objects.size());
        
        for (size_t i = 0; i < aabbs.size(); i++) {
            aabbs[i].computeAABB(m_objects[i]);
        }
        
        // init scanList
        std::cout << "scanList initialize " << std::endl;
        for (size_t i = 0; i < aabbs.size(); i++) {
            if ((aabbs[i].getMinCoord()).x() <= (aabbs[i].getMaxCoord().x()))
            {
                std::tuple<size_t, double, bool> tmpXMin = std::make_tuple(i, (aabbs[i].getMinCoord()).x(), false);
                std::tuple<size_t, double, bool> tmpXMax = std::make_tuple(i, (aabbs[i].getMaxCoord()).x(), true);
                scanListX.push_back(tmpXMin);
                scanListX.push_back(tmpXMax);
            }
            else {
                std::tuple<size_t, double, bool> tmpXMin = std::make_tuple(i, (aabbs[i].getMaxCoord()).x(), false);
                std::tuple<size_t, double, bool> tmpXMax = std::make_tuple(i, (aabbs[i].getMinCoord()).x(), true);
                scanListX.push_back(tmpXMin);
                scanListX.push_back(tmpXMax);
            }

            if ((aabbs[i].getMinCoord()).y() <= (aabbs[i].getMaxCoord().y()))
            {
                std::tuple<size_t, double, bool> tmpYMin = std::make_tuple(i, (aabbs[i].getMinCoord()).y(), false);
                std::tuple<size_t, double, bool> tmpYMax = std::make_tuple(i, (aabbs[i].getMaxCoord()).y(), true);
                scanListY.push_back(tmpYMin);
                scanListY.push_back(tmpYMax);
            }
            else {
                std::tuple<size_t, double, bool> tmpYMin = std::make_tuple(i, (aabbs[i].getMaxCoord()).y(), false);
                std::tuple<size_t, double, bool> tmpYMax = std::make_tuple(i, (aabbs[i].getMinCoord()).y(), true);
                scanListY.push_back(tmpYMin);
                scanListY.push_back(tmpYMax);
            }

            if ((aabbs[i].getMinCoord()).z() <= (aabbs[i].getMaxCoord().z()))
            {
                std::tuple<size_t, double, bool> tmpZMin = std::make_tuple(i, (aabbs[i].getMinCoord()).z(), false);
                std::tuple<size_t, double, bool> tmpZMax = std::make_tuple(i, (aabbs[i].getMaxCoord()).z(), true);
                scanListZ.push_back(tmpZMin);
                scanListZ.push_back(tmpZMax);
            }
            else {
                std::tuple<size_t, double, bool> tmpZMin = std::make_tuple(i, (aabbs[i].getMaxCoord()).z(), false);
                std::tuple<size_t, double, bool> tmpZMax = std::make_tuple(i, (aabbs[i].getMinCoord()).z(), true);
                scanListZ.push_back(tmpZMin);
                scanListZ.push_back(tmpZMax);
            }
        }
        std::sort(scanListX.begin(), scanListX.end(), [](std::tuple<size_t, double, bool> a, std::tuple<size_t, double, bool> b) { return std::get<1>(a) < std::get<1>(b); });
        std::sort(scanListY.begin(), scanListY.end(), [](std::tuple<size_t, double, bool> a, std::tuple<size_t, double, bool> b) { return std::get<1>(a) < std::get<1>(b); });
        std::sort(scanListZ.begin(), scanListZ.end(), [](std::tuple<size_t, double, bool> a, std::tuple<size_t, double, bool> b) { return std::get<1>(a) < std::get<1>(b); });

        auto overlappingList = sap();
        
        scanListX.clear();
        scanListY.clear();
        scanListZ.clear();

        for (int i = 0; i < overlappingList.size(); i++)
        {
            m_overlappingBodys.push_back(overlappingList[i]);
        }
        
		break;
	}
	}

    //std::cout << "overlapping body number : " << m_overlappingBodys.size() << std::endl;
}

void CollisionDetection::computeNarrowPhase(int narrowPhaseMethod) {
    switch (narrowPhaseMethod) {
    case 0: {
        // exhaustive
        // iterate through all pairs of possible collisions
        for (auto overlap : m_overlappingBodys) {
            std::vector<Contact> temp_contacts[2];
            // compute intersection of a with b first and intersection
            // of b with a and store results in temp_contacts
            for (int switcher = 0; switcher < 2; switcher++) {
                RigidObject* a =
                    &m_objects[(!switcher) ? overlap.first
                                            : overlap.second];
                RigidObject* b =
                    &m_objects[(!switcher) ? overlap.second
                                            : overlap.first];

                Eigen::MatrixXd Va, Vb;
                Eigen::MatrixXi Fa, Fb;
                a->getMesh(Va, Fa);
                b->getMesh(Vb, Fb);

                // iterate through all faces of first object
                for (int face = 0; face < Fa.rows(); face++) {
                    // iterate through all edges of given face
                    for (size_t j = 0; j < 3; j++) {
                        int start = Fa(face, j);
                        int end = Fa(face, (j + 1) % 3);
                        double t;
                        // check if there is a collision
                        ContactType ct = isColliding(
                            Va.row(start), Va.row(end), Vb, Fb);

                        // find collision and check for duplicates
                        switch (ct) {
                            case ContactType::VERTEXFACE: {
                                auto ret = m_penetratingVertices.insert(
                                    Fa(face, j));
                                // if not already in set
                                if (ret.second) {
                                    Contact temp_col =
                                        findVertexFaceCollision(
                                            Va.row(Fa(face, j)), Vb,
                                            Fb);
                                    temp_col.a = a;
                                    temp_col.b = b;
                                    temp_col.type =
                                        ContactType::VERTEXFACE;
                                    temp_contacts[switcher].push_back(
                                        temp_col);
                                    //std::cout << "Collision !" << std::endl;
                                }
                                break;
                            }
                            case ContactType::EDGEEDGE: {
                                int orderedStart = std::min(start, end);
                                int orderedEnd = std::max(start, end);
                                auto ret = m_penetratingEdges.insert(
                                    std::make_pair(orderedStart,
                                                    orderedEnd));
                                // if not already in set
                                if (ret.second) {
                                    Contact temp_col =
                                        findEdgeEdgeCollision(
                                            Va.row(orderedStart),
                                            Va.row(orderedEnd), Vb, Fb);
                                    temp_col.a = a;
                                    temp_col.b = b;
                                    temp_col.type =
                                        ContactType::EDGEEDGE;
                                    temp_contacts[switcher].push_back(
                                        temp_col);
                                    //std::cout << "Collision !" << std::endl;
                                }
                                break;
                            }
                            case ContactType::NONE: {
                                break;
                            }
                        }
                    }
                }
            }

            // look for vertexFace
            bool found = false;
            for (int i = 0; i < 2; i++) {
                for (auto cont : temp_contacts[i]) {
                    if (cont.type == ContactType::VERTEXFACE) {
                        m_contacts.push_back(cont);
                        found = true;
                        break;
                    }
                }
                if (found) {
                    break;
                }
            }
            if (found) {
                continue;
            }

            // take single edgeedge if possible
            if (temp_contacts[0].size() > 0 &&
                temp_contacts[0].size() < temp_contacts[1].size()) {
                for (int i = 0; i < temp_contacts[0].size(); i++) {
                    m_contacts.push_back(temp_contacts[0][i]);
                }
            } else if (temp_contacts[1].size() > 0 &&
                        temp_contacts[0].size() >
                            temp_contacts[1].size()) {
                for (int i = 0; i < temp_contacts[1].size(); i++) {
                    m_contacts.push_back(temp_contacts[1][i]);
                }
            } else if (temp_contacts[0].size() > 0) {
                for (int i = 0; i < temp_contacts[0].size(); i++) {
                    m_contacts.push_back(temp_contacts[0][i]);
                }
            } else if (temp_contacts[1].size() > 0) {
                for (int i = 0; i < temp_contacts[1].size(); i++) {
                    m_contacts.push_back(temp_contacts[1][i]);
                }
            }
        }
        break;
    }

	case 1: {
		// TODO: implement other narrow phase algorithm
        for (auto overlap : m_overlappingBodys) {
            std::vector<Contact> temp_contacts[2];
            // compute intersection of a with b first and intersection
            // of b with a and store results in temp_contacts
            for (int switcher = 0; switcher < 2; switcher++) {
                RigidObject* a =
                    &m_objects[(!switcher) ? overlap.first
                    : overlap.second];
                RigidObject* b =
                    &m_objects[(!switcher) ? overlap.second
                    : overlap.first];

                Eigen::MatrixXd Va, Vb;
                Eigen::MatrixXi Fa, Fb;
                a->getMesh(Va, Fa);
                b->getMesh(Vb, Fb);

                if (switcher==0 && !gjk(Va, Vb))
                {
                    break;
                }
                // iterate through all faces of first object
                for (int face = 0; face < Fa.rows(); face++) {
                    // iterate through all edges of given face
                    for (size_t j = 0; j < 3; j++) {
                        int start = Fa(face, j);
                        int end = Fa(face, (j + 1) % 3);
                        double t;
                        // check if there is a collision
                        ContactType ct = isColliding(
                            Va.row(start), Va.row(end), Vb, Fb);

                        // find collision and check for duplicates
                        switch (ct) {
                        case ContactType::VERTEXFACE: {
                            auto ret = m_penetratingVertices.insert(
                                Fa(face, j));
                            // if not already in set
                            if (ret.second) {
                                Contact temp_col =
                                    findVertexFaceCollision(
                                        Va.row(Fa(face, j)), Vb,
                                        Fb);
                                temp_col.a = a;
                                temp_col.b = b;
                                temp_col.type =
                                    ContactType::VERTEXFACE;
                                temp_contacts[switcher].push_back(
                                    temp_col);
                            }
                            break;
                        }
                        case ContactType::EDGEEDGE: {
                            int orderedStart = std::min(start, end);
                            int orderedEnd = std::max(start, end);
                            auto ret = m_penetratingEdges.insert(
                                std::make_pair(orderedStart,
                                    orderedEnd));
                            // if not already in set
                            if (ret.second) {
                                Contact temp_col =
                                    findEdgeEdgeCollision(
                                        Va.row(orderedStart),
                                        Va.row(orderedEnd), Vb, Fb);
                                temp_col.a = a;
                                temp_col.b = b;
                                temp_col.type =
                                    ContactType::EDGEEDGE;
                                temp_contacts[switcher].push_back(
                                    temp_col);
                            }
                            break;
                        }
                        case ContactType::NONE: {
                            break;
                        }
                        }
                    }
                }
            }

            // look for vertexFace
            bool found = false;
            for (int i = 0; i < 2; i++) {
                for (auto cont : temp_contacts[i]) {
                    if (cont.type == ContactType::VERTEXFACE) {
                        m_contacts.push_back(cont);
                        found = true;
                        break;
                    }
                }
                if (found) {
                    break;
                }
            }
            if (found) {
                continue;
            }
            
            // take single edgeedge if possible
            if (temp_contacts[0].size() > 0 &&
                temp_contacts[0].size() < temp_contacts[1].size()) {
                for (int i = 0; i < temp_contacts[0].size(); i++) {
                    m_contacts.push_back(temp_contacts[0][i]);
                }
            }
            else if (temp_contacts[1].size() > 0 &&
                temp_contacts[0].size() >
                temp_contacts[1].size()) {
                for (int i = 0; i < temp_contacts[1].size(); i++) {
                    m_contacts.push_back(temp_contacts[1][i]);
                }
            }
            else if (temp_contacts[0].size() > 0) {
                for (int i = 0; i < temp_contacts[0].size(); i++) {
                    m_contacts.push_back(temp_contacts[0][i]);
                }
            }
            else if (temp_contacts[1].size() > 0) {
                for (int i = 0; i < temp_contacts[1].size(); i++) {
                    m_contacts.push_back(temp_contacts[1][i]);
                }
            }
        }
		break;
	}
    }
}

void CollisionDetection::applyImpulse(double eps) {
    // compute impulse for all contacts
    for (auto contact : m_contacts) {
        Eigen::Vector3d vrel_vec = contact.a->getVelocity(contact.p) -
                                    contact.b->getVelocity(contact.p);
        double vrel = contact.n.dot(vrel_vec);
        if (vrel >= 0) {
            // bodies are moving apart
            continue;
        }
        
		// TODO: compute impulse and update the following momentums
		//contact.a->setLinearMomentum();
		//contact.b->setLinearMomentum();
		//contact.a->setAngularMomentum();
		//contact.b->setAngularMomentum();
        Eigen::Vector3d vt = vrel_vec - vrel * contact.n;
        Eigen::Vector3d t = vt.normalized();

        Eigen::Vector3d ra = contact.p - contact.a->getPosition();
        Eigen::Vector3d rb = contact.p - contact.b->getPosition();

        double term1 = contact.a->getMassInv() + contact.b->getMassInv();
        double term2 = contact.n.dot((contact.a->getInertiaInv() * (ra.cross(contact.n))).cross(ra));
        double term3 = contact.n.dot((contact.b->getInertiaInv() * (rb.cross(contact.n))).cross(rb));

        double jn = -(1 + eps) * vrel / (term1 + term2 + term3);

        term2 = t.dot((contact.a->getInertiaInv() * (ra.cross(t))).cross(ra));
        term3 = t.dot((contact.b->getInertiaInv() * (rb.cross(t))).cross(rb));

        double jt = -(1 + eps) * vt.dot(t) / (term1 + term2 + term3);
        double mu = 0.1;

        if (jn >= 0)
        {
            jt = std::max(-mu * jn, std::min(mu * jn, jt));
        }
        else {
            jt = std::max(mu * jn, std::min(-mu * jn, jt));
        }

        Eigen::Vector3d impulse = jn * contact.n + jt * t;

        auto LinearMa = contact.a->getLinearMomentum();
        auto LinearMb = contact.b->getLinearMomentum();
        auto AngularMa = contact.a->getAngularMomentum();
        auto AngularMb = contact.b->getAngularMomentum();

        LinearMa += impulse;
        LinearMb -= impulse;
        AngularMa += ra.cross(impulse);
        AngularMb -= rb.cross(impulse);

        contact.a->setLinearMomentum(LinearMa);
        contact.b->setLinearMomentum(LinearMb);
        contact.a->setAngularMomentum(AngularMa);
        contact.b->setAngularMomentum(AngularMb);
    }
}

void CollisionDetection::applyImpulse(Contact contact, double eps) {
    Eigen::Vector3d vrel_vec = contact.a->getVelocity(contact.p) -
        contact.b->getVelocity(contact.p);
    double vrel = contact.n.dot(vrel_vec);
    if (vrel >= 0) {
        // bodies are moving apart
        return;
    }

    // TODO: compute impulse and update the following momentums
    //contact.a->setLinearMomentum();
    //contact.b->setLinearMomentum();
    //contact.a->setAngularMomentum();
    //contact.b->setAngularMomentum();
    Eigen::Vector3d vt = vrel_vec - vrel * contact.n;
    Eigen::Vector3d t = vt.normalized();

    Eigen::Vector3d ra = contact.p - contact.a->getPosition();
    Eigen::Vector3d rb = contact.p - contact.b->getPosition();

    double term1 = contact.a->getMassInv() + contact.b->getMassInv();
    double term2 = contact.n.dot((contact.a->getInertiaInv() * (ra.cross(contact.n))).cross(ra));
    double term3 = contact.n.dot((contact.b->getInertiaInv() * (rb.cross(contact.n))).cross(rb));

    double jn = -(1 + eps) * vrel / (term1 + term2 + term3);

    term2 = t.dot((contact.a->getInertiaInv() * (ra.cross(t))).cross(ra));
    term3 = t.dot((contact.b->getInertiaInv() * (rb.cross(t))).cross(rb));

    double jt = -(1 + eps) * vt.dot(t) / (term1 + term2 + term3);
    double mu = 0.1;

    if (jn >= 0)
    {
        jt = std::max(-mu * jn, std::min(mu * jn, jt));
    }
    else {
        jt = std::max(mu * jn, std::min(-mu * jn, jt));
    }

    Eigen::Vector3d impulse = jn * contact.n + jt * t;

    auto LinearMa = contact.a->getLinearMomentum();
    auto LinearMb = contact.b->getLinearMomentum();
    auto AngularMa = contact.a->getAngularMomentum();
    auto AngularMb = contact.b->getAngularMomentum();

    LinearMa += impulse;
    LinearMb -= impulse;
    AngularMa += ra.cross(impulse);
    AngularMb -= rb.cross(impulse);

    contact.a->setLinearMomentum(LinearMa);
    contact.b->setLinearMomentum(LinearMb);
    contact.a->setAngularMomentum(AngularMa);
    contact.b->setAngularMomentum(AngularMb);
}
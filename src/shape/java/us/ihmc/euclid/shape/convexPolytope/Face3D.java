package us.ihmc.euclid.shape.convexPolytope;

import java.util.Collection;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.shape.convexPolytope.impl.AbstractFace3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Implementation of a face 3D that belongs to a convex polytope 3D.
 * <p>
 * This is part of a Doubly Connected Edge List data structure
 * <a href="https://en.wikipedia.org/wiki/Doubly_connected_edge_list"> link</a>.
 * </p>
 *
 * @author Apoorv Shrivastava
 * @author Sylvain Bertrand
 */
public class Face3D extends AbstractFace3D<Vertex3D, HalfEdge3D, Face3D>
{
   /** The normal vector of the support plane of this face. */
   private final Vector3D normal = new Vector3D();
   /** The centroid of this face. */
   private final Point3D centroid = new Point3D();
   /** The tightest bounding box entirely containing this face. */
   private final BoundingBox3D boundingBox = new BoundingBox3D();

   /**
    * Creates a new empty face.
    *
    * @param initialGuessNormal initial guess for what this face's normal should be. Not modified.
    */
   public Face3D(Vector3DReadOnly initialGuessNormal)
   {
      super(HalfEdge3D::new);
      initialize(initialGuessNormal);
   }

   /**
    * Creates a new empty face.
    *
    * @param initialGuessNormal  initial guess for what this face's normal should be. Not modified.
    * @param constructionEpsilon tolerance used when adding vertices to a face to trigger a series of
    *                            edge-cases.
    */
   public Face3D(Vector3DReadOnly initialGuessNormal, double constructionEpsilon)
   {
      super(HalfEdge3D::new, constructionEpsilon);
      initialize(initialGuessNormal);
   }

   /**
    * Creates a new face given its edges.
    *
    * @param faceEdges           the edges composing the new face. Not modified, reference to the edges
    *                            saved.
    * @param normal              the face's normal. Not modified.
    * @param constructionEpsilon tolerance used when adding vertices to a face to trigger a series of
    *                            edge-cases.
    */
   public Face3D(Collection<HalfEdge3D> faceEdges, Vector3DReadOnly normal, double constructionEpsilon)
   {
      super(HalfEdge3D::new, constructionEpsilon);
      initialize(faceEdges, normal);
   }

   @Override
   public Point3D getCentroid()
   {
      return centroid;
   }

   @Override
   public Vector3D getNormal()
   {
      return normal;
   }
   
   @Override
   public BoundingBox3D getBoundingBox()
   {
      return boundingBox;
   }
}

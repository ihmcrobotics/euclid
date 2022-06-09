package us.ihmc.euclid.shape.convexPolytope;

import java.util.List;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.shape.convexPolytope.impl.AbstractConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.ConvexPolytope3DTroublesomeDataset;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeConstructionTools;
import us.ihmc.euclid.tuple3D.Point3D;

/**
 * Implementation of a convex polytope 3D.
 * <p>
 * This is part of a Doubly Connected Edge List data structure
 * <a href="https://en.wikipedia.org/wiki/Doubly_connected_edge_list"> link</a>.
 * </p>
 *
 * @author Apoorv Shrivastava
 * @author Sylvain Bertrand
 */
public class ConvexPolytope3D extends AbstractConvexPolytope3D<Vertex3D, HalfEdge3D, Face3D>
{
   /** The centroid of this convex polytope. */
   private final Point3D centroid = new Point3D();
   /** The tightest bounding box entirely containing this face. */
   private final BoundingBox3D boundingBox = new BoundingBox3D();

   /**
    * Creates a new empty convex polytope.
    */
   public ConvexPolytope3D()
   {
      this(EuclidPolytopeConstructionTools.DEFAULT_CONSTRUCTION_EPSILON);
   }

   /**
    * Creates a new empty convex polytope.
    *
    * @param constructionEpsilon tolerance used when adding vertices to a convex polytope to trigger a
    *                            series of edge-cases.
    */
   public ConvexPolytope3D(double constructionEpsilon)
   {
      super(constructionEpsilon);
      setFactories(Vertex3D::new, HalfEdge3D::new, Face3D::new);
      initialize();
   }

   /**
    * Creates a new convex polytope and adds vertices provided by the given supplier.
    *
    * @param vertex3DSupplier the vertex supplier to get the vertices to add to this convex polytope.
    */
   public ConvexPolytope3D(Vertex3DSupplier vertex3DSupplier)
   {
      this(vertex3DSupplier, EuclidPolytopeConstructionTools.DEFAULT_CONSTRUCTION_EPSILON);
   }

   /**
    * Creates a new convex polytope and adds vertices provided by the given supplier.
    *
    * @param vertex3DSupplier    the vertex supplier to get the vertices to add to this convex
    *                            polytope.
    * @param constructionEpsilon tolerance used when adding vertices to a convex polytope to trigger a
    *                            series of edge-cases.
    */
   public ConvexPolytope3D(Vertex3DSupplier vertex3DSupplier, double constructionEpsilon)
   {
      this(constructionEpsilon);
      addVertices(vertex3DSupplier);
   }

   /**
    * Creates a new convex polytope identical to {@code other}.
    *
    * @param other the other convex polytope to copy. Not modified.
    */
   public ConvexPolytope3D(ConvexPolytope3DReadOnly other)
   {
      this(other.getConstructionEpsilon());
      set(other);
   }

   /**
    * Creates a new convex polytope and initializes its faces.
    * <p>
    * This constructor should only be used with {@link ConvexPolytope3DTroublesomeDataset}.
    * </p>
    *
    * @param faces               the faces composing the new convex polytope. Not modified, faces
    *                            reference saved.
    * @param constructionEpsilon tolerance used when adding vertices to a convex polytope to trigger a
    *                            series of edge-cases.
    */
   public ConvexPolytope3D(List<Face3D> faces, double constructionEpsilon)
   {
      this(constructionEpsilon);
      initialize(faces);
   }

   /**
    * Sets this convex polytope to be identical to {@code other}.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    */
   public void set(ConvexPolytope3D other)
   {
      this.set((ConvexPolytope3DReadOnly) other);
   }

   @Override
   public Point3D getCentroid()
   {
      return centroid;
   }

   @Override
   public BoundingBox3DBasics getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public ConvexPolytope3D copy()
   {
      return new ConvexPolytope3D(this);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(ConvexPolytope3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof ConvexPolytope3DReadOnly)
         return equals((ConvexPolytope3DReadOnly) object);
      else
         return false;
   }
}

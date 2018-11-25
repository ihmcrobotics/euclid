package us.ihmc.euclid.shape.convexPolytope;

import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytopeFaceProvider;
import us.ihmc.euclid.shape.convexPolytope.interfaces.PolytopeListener;
import us.ihmc.euclid.shape.convexPolytope.interfaces.PolytopeVertexProvider;

/**
 * A convex polytope is a collection of faces that describe it
 * 
 * This class is a data structure for storing a polytope in the DCEL notation (ref:
 * https://en.wikipedia.org/wiki/Doubly_connected_edge_list). Based on the original implementation
 * by Jerry Pratt
 * 
 * @author Apoorv S
 */

public class ExtendedConvexPolytope extends ConvexPolytopeBasics
{
   private final ConvexPolytopeFaceBuilder faceBuilder = new ConvexPolytopeFaceBuilder();
   private final PolytopeVertexBuilder vertexBuilder = new PolytopeVertexBuilder();

   public ExtendedConvexPolytope()
   {
      super();
   }

   public ExtendedConvexPolytope(PolytopeListener listener)
   {
      super(listener);
   }

   public ExtendedConvexPolytope(ExtendedConvexPolytope polytope)
   {
      super(polytope);
   }

   @Override
   protected PolytopeVertexProvider getVertexProvider()
   {
      return vertexBuilder;
   }

   @Override
   protected ConvexPolytopeFaceProvider getConvexFaceProvider()
   {
      return faceBuilder;
   }

   @Override
   public Face3D getFace(int index)
   {
      return (Face3D) super.getFace(index);
   }
}

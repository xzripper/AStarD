/**
 * This is a heap implementation that can be used as a Min-Heap or Max-Heap.
 * It keeps the smallest or biggest item at the top depending on the type of heap.
 * The heap can be built from an already existing array or can be initially empty.
 * New items can be added, and the data structure will rearrange itself to maintain order.
 * Popping the top element replaces it with the last and reorders the heap.
 * It is iterable and optimized to avoid useless allocations of memory.
 * Uses safe code and minimizes reliance on the garbage collector.
 *
 * https://github.com/xzripper/DHeap by Évan
*/

module DHeap.Heap;

enum HeapType { MAX_HEAP, MIN_HEAP }

struct Heap(T)
{
    private T[] m_HeapData;

    private HeapType m_HeapType;

    this( T[] p_HeapInitObjs, HeapType p_HeapType ) @safe
    {
        m_HeapData = p_HeapInitObjs;
        m_HeapType = p_HeapType;

        for(int t_Idx=((cast(int) m_HeapData.length) / 2) - 1; t_Idx >= 0; t_Idx--)
        {
            _HeapSiftDown( t_Idx );
        }
    }

    this( HeapType p_HeapType ) @safe @nogc
    {
        m_HeapType = p_HeapType;
    }

    void HeapPush( T p_HeapObject ) @safe
    {
        m_HeapData ~= p_HeapObject;

        _HeapSiftUp( m_HeapData.length - 1 );
    }

    T HeapPop() @safe
    {
        T t_HeapFront = m_HeapData[0];

        m_HeapData[0] = m_HeapData[$-1];
        m_HeapData = m_HeapData[0..$-1];

        _HeapSiftDown( 0 );

        return t_HeapFront;
    }

    private void _HeapSiftUp( size_t p_Idx ) @safe {
        while( p_Idx > 0 )
        {
            size_t t_HeapParent = (p_Idx - 1) / 2;

            if( _Compare( m_HeapData[p_Idx], m_HeapData[t_HeapParent] ) )
            {
                _Swap( m_HeapData[p_Idx], m_HeapData[t_HeapParent] );

                p_Idx = t_HeapParent;
            } else { break; }
        }
    }

    private void _HeapSiftDown( size_t p_Idx ) @safe {
        while( 1 )
        {
            size_t t_HeapLeft = 2 * p_Idx + 1;
            size_t t_HeapRight = 2 * p_Idx + 2;

            size_t t_HeapTarget = p_Idx;

            if( t_HeapLeft < m_HeapData.length && _Compare( m_HeapData[t_HeapLeft], m_HeapData[t_HeapTarget] ) )
            {
                t_HeapTarget = t_HeapLeft;
            }

            if( t_HeapRight < m_HeapData.length && _Compare( m_HeapData[t_HeapRight], m_HeapData[t_HeapTarget] ) )
            {
                t_HeapTarget = t_HeapRight;
            }

            if( t_HeapTarget == p_Idx )
            {
                break;
            }

            _Swap( m_HeapData[p_Idx], m_HeapData[t_HeapTarget] );

            p_Idx = t_HeapTarget;
        }
    }

    private void _Swap( ref T p_Object0, ref T p_Object1 ) @safe @nogc
    {
        T t_Tmp0 = p_Object0;

        p_Object0 = p_Object1;
        p_Object1 = t_Tmp0;
    }

    private bool _Compare( T p_Object0, T p_Object1 ) @safe @nogc
    {
        return (m_HeapType == HeapType.MIN_HEAP) ? p_Object0 < p_Object1 : p_Object0 > p_Object1;
    }

    T HeapFront() const @property @safe @nogc
    {
        return m_HeapData[0];
    }

    bool Empty() const @property @safe @nogc
    {
        return m_HeapData.length == 0;
    }

    size_t Size() const @property @safe @nogc
    {
        return m_HeapData.length;
    }

    int opApply( int delegate(ref T) HEAP_DELEGATE )
    {
        foreach( ref t_HeapObject; m_HeapData )
        {
            auto t_DelegateOut = HEAP_DELEGATE( t_HeapObject );

            if( t_DelegateOut ) {
                return t_DelegateOut;
            }
        }

        return 0;
    }
}

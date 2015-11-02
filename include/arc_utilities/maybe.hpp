#ifndef MAYBE_HPP
#define MAYBE_HPP

#include <assert.h>

// TODO: add some comments to all this code
namespace Maybe
{
    template <typename T>
    class Maybe
    {
        public:
            Maybe()
                : maybe_( false )
            {}

            Maybe( const T& val )
                : maybe_( true )
                , val_( val )
            {}

            Maybe( T&& val )
                : maybe_( true )
                , val_( val )
            {}

            bool valid()
            {
                return maybe_;
            }

            T& get()
            {
                assert( maybe_ );
                return val_;
            }

            Maybe& operator=( const T& val )
            {
                maybe_ = true;
                val_ = val;
                return *this;
            }

            Maybe& operator=( T&& val )
            {
                maybe_ = true;
                val_ = val;
                return *this;
            }


        private:
            bool maybe_;
            T val_;
    };
}

#endif

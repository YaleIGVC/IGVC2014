:map <F9> :AT<CR>
syntax enable
set softtabstop=4
set smartindent
set tabstop=4
set shiftwidth=4
set expandtab

" Make the mouse do magick
set mouse=a
"
" Set statusline, for SmartusLine plugin
set statusline=%<%f\ %h%m%r%=%-14.(%l,%c%V%)\ %P
set laststatus=1
set nocompatible

inoremap <S-Insert> <MiddleMouse>
set number          " show line numbers
set ruler           " show ruler
set showcmd         " show command line
set showmatch
inoremap <expr> <C-j>     pumvisible() ? "\<C-n>" : "\<C-j>"
inoremap <expr> <C-k>       pumvisible() ? "\<C-p>" : "\<C-k>"
set incsearch       " incremental search
set hlsearch        " highlight search results (":nohlsearch" unhighlights)
set autoread
":autocmd BufEnter * lcd %:p:h " change directory to current buffer directory
:cab e  tabe
filetype plugin indent on
map <C-H> <C-W>h<C-W>_
map <C-L> <C-W>l<C-W>_
map <C-J> <C-W>j<C-W>_
map <C-K> <C-W>k<C-W>_
:map <F8> :TlistToggle<CR>
map <silent><A-Right> :tabnext<CR>
map <silent><A-Left> :tabprevious<CR>

" omnicppcomplete options
map <C-x><C-x><C-T> :!ctags -R --c++-kinds=+p --fields=+iaS --extra=+q -f ~/.vim/commontags /usr/include /usr/local/include <CR><CR>
"set tags+=~/.vim/commontags
set tags+=$HOME/.vim/tags/python.ctags
map <silent><C-Left> <C-T>
map <silent><C-Right> <C-]>

"set tags+=./tags;/ 

" --- OmniCppComplete ---
" -- required --
set nocp " non vi compatible mode
filetype plugin on " enable plugins

" -- optional --
" auto close options when exiting insert mode or moving away
"autocmd CursorMovedI * if pumvisible() == 0|pclose|endif
"utocmd InsertLeave * if pumvisible() == 0|pclose|endif
"et completeopt=menu,menuone

" -- configs --
"et OmniCpp_MayCompleteDot = 1 " autocomplete with .
"et OmniCpp_MayCompleteArrow = 1 " autocomplete with ->
"et OmniCpp_MayCompleteScope = 1 " autocomplete with ::
"et OmniCpp_SelectFirstItem = 2 " select first item (but don't insert)
let OmniCpp_NamespaceSearch = 2 " search namespaces in this and included files
"et OmniCpp_ShowPrototypeInAbbr = 1 " show function prototype (i.e. parameters) in popup window
"et OmniCpp_LocalSearchDecl = 1 " don't require special style of function opening braces

" -- ctags --
" map <ctrl>+F12 to generate ctags for current folder:
map <F7> :!cctags<CR>

" Setup the tab key to do autocompletion
"function! CompleteTab()
"let prec = strpart( getline('.'), 0, col('.')-1 )
"if prec =~ '^\s*$' || prec =~ '\s$'
 "return "\<tab>"
"else
 "return "\<c-x>\<c-o>"
"endif
"endfunction
"inoremap <tab> <c-r>=CompleteTab()<cr>
autocmd FileType python set omnifunc=pythoncomplete#Complete
inoremap <Nul> <C-X><C-o>
function! My_TabComplete()
    let line = getline('.')                         " curline
    let substr = strpart(line, -1, col('.')+1)      " from start to cursor
    let substr = matchstr(substr, "[^ \t]*$")       " word till cursor
    if (strlen(substr)==0)                          " nothing to match on
        empty string
        return "\<tab>"
    endif
    let bool = match(substr, '\.')                  " position of period, if
    if (bool==-1)
        return "\<C-X>\<C-P>"                         " existing text matching
    else
        return "\<C-X>\<C-U>"                         " plugin matching 
    endif
endfunction

" From http://www.codeography.com/2013/06/19/navigating-vim-and-tmux-splits.html
if exists('$TMUX')
  function! TmuxOrSplitSwitch(wincmd, tmuxdir)
    let previous_winnr = winnr()
    execute "wincmd " . a:wincmd
    if previous_winnr == winnr()
      " The sleep and & gives time to get back to vim so tmux's focus tracking
      " can kick in and send us our ^[[O
      execute "silent !sh -c 'sleep 0.01; tmux select-pane -" . a:tmuxdir . "' &"
      redraw!
    endif
  endfunction
  let previous_title = substitute(system("tmux display-message -p '#{pane_title}'"), '\n', '', '')
  let &t_ti = "\<Esc>]2;vim\<Esc>\\" . &t_ti
  let &t_te = "\<Esc>]2;". previous_title . "\<Esc>\\" . &t_te
  nnoremap <silent> <C-h> :call TmuxOrSplitSwitch('h', 'L')<cr>
  nnoremap <silent> <C-j> :call TmuxOrSplitSwitch('j', 'D')<cr>
  nnoremap <silent> <C-k> :call TmuxOrSplitSwitch('k', 'U')<cr>
  nnoremap <silent> <C-l> :call TmuxOrSplitSwitch('l', 'R')<cr>
else
  map <C-h> <C-w>h
  map <C-j> <C-w>j
  map <C-k> <C-w>k
  map <C-l> <C-w>l
endif

" Perform some sort of autocomplete magicks
"autocmd BufNew,BufRead *.java inoremap <tab> <C-R>=My_TabComplete()<CR>



" REQUIRED. This makes vim invoke Latex-Suite when you open a tex file.
"filetype plugin on

" IMPORTANT: win32 users will need to have 'shellslash' set so that latex
" can be called correctly.
set shellslash

" IMPORTANT: grep will sometimes skip displaying the file name if you
" search in a singe file. This will confuse Latex-Suite. Set your grep
" program to always generate a file-name.
set grepprg=grep\ -nH\ $*

" OPTIONAL: This enables automatic indentation as you type.
"filetype indent on

" OPTIONAL: Starting with Vim 7, the filetype of empty .tex files defaults to
" 'plaintex' instead of 'tex', which results in vim-latex not being loaded.
" The following changes the default filetype back to 'tex':
let g:tex_flavor='latex'

" For making scrolling happen more quickly
" " normal mode:
nnoremap <c-j> 5j
nnoremap <c-k> 5k
" visual mode:
xnoremap <c-j> 5j
xnoremap <c-k> 5k

colorscheme desert
"colorscheme lucius

" ROS Stuff
au BufRead,BufNewFile *.launch setfiletype xml
